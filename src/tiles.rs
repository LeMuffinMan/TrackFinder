//! Source de tuiles (abstraction) + cache asynchrone.
//!
//! Une source répond à « donne-moi les octets pour ce z/x/y ». HTTP aujourd'hui ;
//! fichiers locaux ou cache hors-ligne plus tard, sans toucher au reste.

use std::collections::{HashMap, HashSet};
use std::hash::Hash;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Description d'une tuile
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RasterLayer {
    PlanIgn,
    Contours,
    Slopes,
    Ortho,
}

impl RasterLayer {
    pub fn wmts_name(self) -> &'static str {
        match self {
            RasterLayer::PlanIgn => "GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2",
            RasterLayer::Contours => "ELEVATION.CONTOUR.LINE",
            RasterLayer::Slopes => "GEOGRAPHICALGRIDSYSTEMS.SLOPES.MOUNTAIN",
            RasterLayer::Ortho => "ORTHOIMAGERY.ORTHOPHOTOS",
        }
    }

    pub fn format(self) -> &'static str {
        match self {
            RasterLayer::Ortho => "image/jpeg",
            _ => "image/png",
        }
    }

    /// Plafonds de zoom relevés dans le GetCapabilities : au-delà, la couche
    /// n'existe plus et il faut retomber sur la tuile parente agrandie.
    pub fn zoom_range(self) -> (u8, u8) {
        match self {
            RasterLayer::PlanIgn => (0, 19),
            RasterLayer::Contours => (6, 18),
            RasterLayer::Slopes => (0, 17),
            RasterLayer::Ortho => (0, 19),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            RasterLayer::PlanIgn => "Plan IGN",
            RasterLayer::Contours => "Courbes de niveau",
            RasterLayer::Slopes => "Pentes",
            RasterLayer::Ortho => "Ortho-photo",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Dataset {
    Raster(RasterLayer),
    /// MNT haute résolution, grille WGS84G, BIL 32 bits compressé zlib.
    Elevation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileDesc {
    pub dataset: Dataset,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

// ---------------------------------------------------------------------------
// Abstraction
// ---------------------------------------------------------------------------

pub type TileBytes = Result<Vec<u8>, String>;
pub type TileCallback = Box<dyn FnOnce(TileBytes) + Send + 'static>;

pub trait TileSource {
    fn request(&self, desc: TileDesc, done: TileCallback);
}

/// Source HTTP : Géoplateforme IGN, WMTS, sans clé (CORS `*` vérifié).
pub struct HttpTileSource {
    base: String,
}

impl Default for HttpTileSource {
    fn default() -> Self {
        Self {
            base: "https://data.geopf.fr/wmts".to_owned(),
        }
    }
}

impl HttpTileSource {
    pub fn url(&self, desc: TileDesc) -> String {
        let (layer, matrix_set, format) = match desc.dataset {
            Dataset::Raster(l) => (l.wmts_name(), "PM", l.format()),
            Dataset::Elevation => (
                "ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES",
                "WGS84G",
                "image/x-bil;bits=32",
            ),
        };
        format!(
            "{base}?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile&LAYER={layer}&STYLE=normal\
&TILEMATRIXSET={matrix_set}&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&FORMAT={format}",
            base = self.base,
            z = desc.z,
            y = desc.y,
            x = desc.x,
        )
    }
}

impl TileSource for HttpTileSource {
    fn request(&self, desc: TileDesc, done: TileCallback) {
        let request = ehttp::Request::get(self.url(desc));
        ehttp::fetch(request, move |result| {
            let bytes = match result {
                Err(e) => Err(e),
                Ok(resp) if !resp.ok => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Ok(resp) => {
                    // Le WMTS renvoie ses erreurs en XML avec un code 200 :
                    // décoder ça comme un PNG donne une erreur incompréhensible.
                    let ct = resp.content_type().unwrap_or_default().to_owned();
                    if ct.contains("xml") || ct.contains("text/html") {
                        Err(format!(
                            "réponse non-tuile ({ct}) : {}",
                            String::from_utf8_lossy(&resp.bytes[..resp.bytes.len().min(200)])
                        ))
                    } else {
                        Ok(resp.bytes)
                    }
                }
            };
            done(bytes);
        });
    }
}

// ---------------------------------------------------------------------------
// Cache asynchrone
// ---------------------------------------------------------------------------

/// Cache générique : demande les octets à la source, les décode à l'arrivée,
/// garde la valeur décodée, évince les moins récemment utilisées.
pub struct TileCache<K: Eq + Hash + Clone + Send + 'static, V> {
    source: Rc<dyn TileSource>,
    inbox: Arc<Mutex<Vec<(K, TileBytes)>>>,
    ready: HashMap<K, V>,
    pending: HashSet<K>,
    failed: HashMap<K, String>,
    last_used: HashMap<K, u64>,
    clock: u64,
    capacity: usize,
}

impl<K: Eq + Hash + Clone + Send + 'static, V> TileCache<K, V> {
    pub fn new(source: Rc<dyn TileSource>, capacity: usize) -> Self {
        Self {
            source,
            inbox: Arc::new(Mutex::new(Vec::new())),
            ready: HashMap::new(),
            pending: HashSet::new(),
            failed: HashMap::new(),
            last_used: HashMap::new(),
            clock: 0,
            capacity,
        }
    }

    pub fn tick(&mut self) {
        self.clock += 1;
    }

    /// Consomme les réponses arrivées depuis la dernière frame.
    pub fn pump(&mut self, mut decode: impl FnMut(&K, &[u8]) -> Result<V, String>) {
        let arrived: Vec<(K, TileBytes)> = {
            let mut inbox = self.inbox.lock().unwrap();
            std::mem::take(&mut *inbox)
        };
        for (key, bytes) in arrived {
            self.pending.remove(&key);
            match bytes.and_then(|b| decode(&key, &b)) {
                Ok(value) => {
                    self.ready.insert(key.clone(), value);
                    self.last_used.insert(key, self.clock);
                }
                Err(e) => {
                    log::warn!("tuile en échec : {e}");
                    self.failed.insert(key, e);
                }
            }
        }
    }

    /// Vrai si la tuile est disponible ; déclenche le fetch si elle est inconnue.
    pub fn ensure(&mut self, key: &K, desc: TileDesc, ctx: &egui::Context) -> bool {
        if self.ready.contains_key(key) {
            self.last_used.insert(key.clone(), self.clock);
            return true;
        }
        if !self.pending.contains(key) && !self.failed.contains_key(key) {
            self.spawn(key.clone(), desc, ctx);
        }
        false
    }

    /// Consulte sans déclencher de fetch (utilisé par le repli sur tuile parente).
    pub fn peek(&mut self, key: &K) -> Option<&V> {
        if self.ready.contains_key(key) {
            self.last_used.insert(key.clone(), self.clock);
        }
        self.ready.get(key)
    }

    fn spawn(&mut self, key: K, desc: TileDesc, ctx: &egui::Context) {
        self.pending.insert(key.clone());
        let inbox = Arc::clone(&self.inbox);
        // Le Context est cloné AVANT la closure : sans request_repaint, la tuile
        // n'apparaîtrait qu'au prochain mouvement de souris.
        let ctx = ctx.clone();
        self.source.request(
            desc,
            Box::new(move |bytes| {
                inbox.lock().unwrap().push((key, bytes));
                ctx.request_repaint();
            }),
        );
    }

    /// Évince les tuiles décodées les moins récemment utilisées.
    pub fn evict(&mut self) {
        if self.ready.len() <= self.capacity {
            return;
        }
        let mut ages: Vec<(u64, K)> = self
            .ready
            .keys()
            .map(|k| (*self.last_used.get(k).unwrap_or(&0), k.clone()))
            .collect();
        ages.sort_by_key(|(age, _)| *age);
        let drop_count = self.ready.len() - self.capacity;
        for (_, key) in ages.into_iter().take(drop_count) {
            self.ready.remove(&key);
            self.last_used.remove(&key);
        }
    }

    /// (prêtes, en cours, en échec)
    pub fn stats(&self) -> (usize, usize, usize) {
        (self.ready.len(), self.pending.len(), self.failed.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// L'URL produite doit être exactement celle vérifiée le 02/09/2026 (200 OK).
    #[test]
    fn url_fond_planign() {
        let s = HttpTileSource::default();
        let url = s.url(TileDesc {
            dataset: Dataset::Raster(RasterLayer::PlanIgn),
            z: 14,
            x: 8504,
            y: 5833,
        });
        assert_eq!(
            url,
            "https://data.geopf.fr/wmts?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile\
&LAYER=GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2&STYLE=normal&TILEMATRIXSET=PM\
&TILEMATRIX=14&TILEROW=5833&TILECOL=8504&FORMAT=image/png"
        );
    }

    #[test]
    fn url_mnt() {
        let s = HttpTileSource::default();
        let url = s.url(TileDesc {
            dataset: Dataset::Elevation,
            z: 14,
            x: 17010,
            y: 4012,
        });
        assert_eq!(
            url,
            "https://data.geopf.fr/wmts?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile\
&LAYER=ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES&STYLE=normal&TILEMATRIXSET=WGS84G\
&TILEMATRIX=14&TILEROW=4012&TILECOL=17010&FORMAT=image/x-bil;bits=32"
        );
    }
}
