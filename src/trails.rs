//! Sentiers OSM : requête Overpass par zone, cache local, index spatial, snap.
//!
//! Le graphe est **local et à la demande** : jamais à l'échelle France. On
//! découpe le monde en zones de `ZONE_DEG`, on ne demande que celles dont on a
//! besoin, et on ne les redemande jamais.

use std::collections::{HashMap, HashSet, VecDeque};
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use crate::geo::LatLon;

/// Deux tailles de zone, en degrés.
///
/// - niveau 0 (~2,2 km) : le clic. Compromis mesuré à Chamonix — 0,03° × 0,04°
///   pèsent 1,15 Mo de JSON, 199 Ko sur le réseau après gzip. Plus grand, la
///   réponse devient pénible en ville ; plus petit, on multiplie les requêtes.
/// - niveau 1 (~11 km) : l'isochrone, qui a besoin de tout un bassin d'un coup.
///   Couvrir 25 km avec des zones de niveau 0 demanderait ~500 requêtes.
///
/// Les deux niveaux alimentent le **même** `TrailNetwork` : l'insertion étant
/// idempotente par identifiant de chemin, le recouvrement ne coûte rien.
pub const ZONE_LEVELS: [f64; 2] = [0.02, 0.10];

/// Taille de la zone du clic — la plus fine.
pub const ZONE_DEG: f64 = ZONE_LEVELS[0];

/// Côté d'une cellule d'index spatial, en degrés (~220 m).
const INDEX_CELL_DEG: f64 = 0.002;

/// Distance maximale acceptée entre un clic et le sentier le plus proche.
pub const SNAP_RADIUS_M: f64 = 60.0;

const HIGHWAY_FILTER: &str =
    "^(path|track|footway|bridleway|steps|cycleway|pedestrian|living_street|unclassified|residential|tertiary)$";

// ---------------------------------------------------------------------------
// Zones
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ZoneKey {
    pub level: u8,
    pub lat: i32,
    pub lon: i32,
}

impl ZoneKey {
    /// Zone fine, celle du clic.
    pub fn of(ll: LatLon) -> Self {
        Self::of_level(ll, 0)
    }

    pub fn of_level(ll: LatLon, level: u8) -> Self {
        let size = ZONE_LEVELS[level as usize];
        Self {
            level,
            lat: (ll.lat / size).floor() as i32,
            lon: (ll.lon / size).floor() as i32,
        }
    }

    pub fn size(self) -> f64 {
        ZONE_LEVELS[self.level as usize]
    }

    /// (sud, ouest, nord, est)
    pub fn bbox(self) -> (f64, f64, f64, f64) {
        let size = self.size();
        let s = self.lat as f64 * size;
        let w = self.lon as f64 * size;
        (s, w, s + size, w + size)
    }
}

// ---------------------------------------------------------------------------
// Données
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WayKind {
    Path,
    Track,
    Footway,
    Steps,
    Cycleway,
    Road,
}

impl WayKind {
    fn from_tag(tag: &str) -> Self {
        match tag {
            "path" | "bridleway" => WayKind::Path,
            "track" => WayKind::Track,
            "footway" | "pedestrian" => WayKind::Footway,
            "steps" => WayKind::Steps,
            "cycleway" => WayKind::Cycleway,
            _ => WayKind::Road,
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            WayKind::Path => egui::Color32::from_rgb(180, 60, 200),
            WayKind::Track => egui::Color32::from_rgb(150, 110, 60),
            WayKind::Footway | WayKind::Steps => egui::Color32::from_rgb(80, 140, 220),
            WayKind::Cycleway => egui::Color32::from_rgb(60, 170, 160),
            WayKind::Road => egui::Color32::from_gray(120),
        }
    }
}

pub struct Way {
    pub id: i64,
    pub kind: WayKind,
    #[allow(dead_code)] // affichage du nom d'itinéraire (M2)
    pub name: Option<String>,
    /// Identifiants des nœuds OSM : deux chemins partageant un nœud sont
    /// connectés. C'est la topologie dont le graphe du M2 aura besoin.
    #[allow(dead_code)]
    pub nodes: Vec<i64>,
    pub points: Vec<LatLon>,
    #[allow(dead_code)] // cotation alpine, overlay « zones dangereuses » (M6)
    pub sac_scale: Option<String>,
    /// (sud, ouest, nord, est) — pré-calculé pour le culling au rendu.
    pub bounds: (f64, f64, f64, f64),
}

impl Way {
    pub fn intersects(&self, view: (f64, f64, f64, f64)) -> bool {
        self.bounds.0 <= view.2 && self.bounds.2 >= view.0 && self.bounds.1 <= view.3 && self.bounds.3 >= view.1
    }
}

/// Position accrochée sur un sentier.
#[derive(Clone, Copy, Debug)]
pub struct Snap {
    pub way_id: i64,
    pub seg: usize,
    /// Position sur le segment, dans [0, 1].
    pub t: f64,
    pub pos: LatLon,
    pub dist_m: f64,
}

// ---------------------------------------------------------------------------
// Réseau
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct TrailNetwork {
    ways: Vec<Way>,
    by_id: HashMap<i64, usize>,
    /// cellule → (indice de chemin, indice de segment)
    index: HashMap<(i32, i32), Vec<(u32, u32)>>,
}

fn cell_of(ll: LatLon) -> (i32, i32) {
    (
        (ll.lat / INDEX_CELL_DEG).floor() as i32,
        (ll.lon / INDEX_CELL_DEG).floor() as i32,
    )
}

/// Mètres par degré, à cette latitude. Sert au calcul de distance point-segment
/// en approximation plane locale — valable à l'échelle d'une cellule.
fn deg_to_m(lat: f64) -> (f64, f64) {
    (111_320.0 * lat.to_radians().cos(), 111_132.0)
}

impl TrailNetwork {
    pub fn ways(&self) -> &[Way] {
        &self.ways
    }

    pub fn way_by_id(&self, id: i64) -> Option<&Way> {
        self.by_id.get(&id).map(|i| &self.ways[*i])
    }

    pub fn len(&self) -> usize {
        self.ways.len()
    }

    pub fn insert(&mut self, way: Way) {
        // Overpass renvoie les chemins entiers, pas découpés à la bbox : deux
        // zones voisines rapportent donc les mêmes chemins de bordure.
        if self.by_id.contains_key(&way.id) || way.points.len() < 2 {
            return;
        }
        let idx = self.ways.len() as u32;
        for (seg, pair) in way.points.windows(2).enumerate() {
            let (a, b) = (cell_of(pair[0]), cell_of(pair[1]));
            for cy in a.0.min(b.0)..=a.0.max(b.0) {
                for cx in a.1.min(b.1)..=a.1.max(b.1) {
                    self.index
                        .entry((cy, cx))
                        .or_default()
                        .push((idx, seg as u32));
                }
            }
        }
        self.by_id.insert(way.id, self.ways.len());
        self.ways.push(way);
    }

    /// Point du réseau le plus proche, dans un rayon donné.
    pub fn snap(&self, ll: LatLon, max_dist_m: f64) -> Option<Snap> {
        let (mx, my) = deg_to_m(ll.lat);
        let cell_m = INDEX_CELL_DEG * my.min(mx);
        let reach = (max_dist_m / cell_m).ceil() as i32 + 1;
        let (cy, cx) = cell_of(ll);

        let mut best: Option<Snap> = None;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let Some(bucket) = self.index.get(&(cy + dy, cx + dx)) else {
                    continue;
                };
                for (wi, si) in bucket {
                    let way = &self.ways[*wi as usize];
                    let a = way.points[*si as usize];
                    let b = way.points[*si as usize + 1];
                    let (t, dist) = point_segment(ll, a, b, mx, my);
                    if dist <= max_dist_m && best.is_none_or(|s| dist < s.dist_m) {
                        best = Some(Snap {
                            way_id: way.id,
                            seg: *si as usize,
                            t,
                            pos: crate::geo::lerp_latlon(a, b, t),
                            dist_m: dist,
                        });
                    }
                }
            }
        }
        best
    }

    /// Géométrie du chemin entre deux positions accrochées **au même chemin OSM**.
    /// C'est le « suivi de tronçon » : deux clics sur le même sentier suivent sa
    /// forme réelle au lieu de la corde.
    pub fn follow(&self, from: &Snap, to: &Snap) -> Option<Vec<LatLon>> {
        if from.way_id != to.way_id {
            return None;
        }
        let way = self.way_by_id(from.way_id)?;
        let forward = (from.seg, from.t) <= (to.seg, to.t);
        let (a, b) = if forward { (from, to) } else { (to, from) };

        let mut out = vec![a.pos];
        for i in (a.seg + 1)..=b.seg {
            out.push(way.points[i]);
        }
        out.push(b.pos);
        if !forward {
            out.reverse();
        }
        Some(out)
    }
}

/// Projection d'un point sur un segment, en approximation plane locale.
/// Renvoie (t dans [0,1], distance en mètres).
fn point_segment(p: LatLon, a: LatLon, b: LatLon, mx: f64, my: f64) -> (f64, f64) {
    let ax = (a.lon - p.lon) * mx;
    let ay = (a.lat - p.lat) * my;
    let bx = (b.lon - p.lon) * mx;
    let by = (b.lat - p.lat) * my;
    let (dx, dy) = (bx - ax, by - ay);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        ((-ax * dx - ay * dy) / len2).clamp(0.0, 1.0)
    };
    let (px, py) = (ax + dx * t, ay + dy * t);
    (t, (px * px + py * py).sqrt())
}

// ---------------------------------------------------------------------------
// Source Overpass
// ---------------------------------------------------------------------------

pub type TrailCallback = Box<dyn FnOnce(Result<String, String>) + Send + 'static>;

pub trait TrailSource {
    /// `attempt` sert au basculement d'instance : Overpass est un service public
    /// à quotas qui répond régulièrement 504 « too busy ».
    fn request(&self, zone: ZoneKey, attempt: usize, done: TrailCallback);
    fn endpoint_count(&self) -> usize;
}

pub struct OverpassSource {
    /// Instances **planétaires** envoyant `Access-Control-Allow-Origin: *`
    /// (vérifié 03/09/2026 — le header n'apparaît que si la requête porte un
    /// `Origin`, donc invisible à `curl` nu).
    ///
    /// ⚠️ `overpass.osm.ch` a bien le CORS mais ne sert qu'un extrait **suisse** :
    /// il répond `200` avec `elements: []` partout ailleurs en France. Un miroir
    /// régional ne se voit pas dans le code d'état — il se voit dans les données.
    /// Ne pas l'ajouter ici.
    endpoints: Vec<String>,
}

impl Default for OverpassSource {
    fn default() -> Self {
        Self {
            endpoints: vec![
                "https://overpass-api.de/api/interpreter".to_owned(),
                "https://overpass.kumi.systems/api/interpreter".to_owned(),
                "https://maps.mail.ru/osm/tools/overpass/api/interpreter".to_owned(),
            ],
        }
    }
}

impl OverpassSource {
    pub fn query(zone: ZoneKey) -> String {
        let (s, w, n, e) = zone.bbox();
        format!(
            "[out:json][timeout:60];\nway[\"highway\"~\"{HIGHWAY_FILTER}\"]({s:.4},{w:.4},{n:.4},{e:.4});\nout geom;"
        )
    }
}

impl TrailSource for OverpassSource {
    fn request(&self, zone: ZoneKey, attempt: usize, done: TrailCallback) {
        let url = self.endpoints[attempt % self.endpoints.len()].clone();
        let mut request = ehttp::Request::post(url, OverpassSource::query(zone).into_bytes());
        request
            .headers
            .insert("Content-Type", "text/plain;charset=UTF-8");
        ehttp::fetch(request, move |result| {
            let body = match result {
                Err(e) => Err(e),
                Ok(resp) if !resp.ok => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Ok(resp) => resp
                    .text()
                    .map(|t| t.to_owned())
                    .ok_or_else(|| "réponse non textuelle".to_owned()),
            };
            done(body);
        });
    }

    fn endpoint_count(&self) -> usize {
        self.endpoints.len()
    }
}

// ---------------------------------------------------------------------------
// Analyse de la réponse
// ---------------------------------------------------------------------------

pub fn parse_overpass(body: &str) -> Result<Vec<Way>, String> {
    let root: serde_json::Value =
        serde_json::from_str(body).map_err(|e| format!("JSON Overpass : {e}"))?;
    // Overpass renvoie ses erreurs en HTML avec un code 200 dans certains cas ;
    // on tombe alors sur l'erreur de parse ci-dessus, pas sur un silence.
    let elements = root
        .get("elements")
        .and_then(|e| e.as_array())
        .ok_or_else(|| "pas de champ `elements`".to_owned())?;

    let mut ways = Vec::with_capacity(elements.len());
    for el in elements {
        if el.get("type").and_then(|t| t.as_str()) != Some("way") {
            continue;
        }
        let Some(id) = el.get("id").and_then(|v| v.as_i64()) else {
            continue;
        };
        let Some(geometry) = el.get("geometry").and_then(|g| g.as_array()) else {
            continue;
        };
        let points: Vec<LatLon> = geometry
            .iter()
            .filter_map(|p| {
                Some(LatLon::new(
                    p.get("lat")?.as_f64()?,
                    p.get("lon")?.as_f64()?,
                ))
            })
            .collect();
        if points.len() < 2 {
            continue;
        }
        let nodes = el
            .get("nodes")
            .and_then(|n| n.as_array())
            .map(|a| a.iter().filter_map(|v| v.as_i64()).collect())
            .unwrap_or_default();
        let tags = el.get("tags");
        let tag = |k: &str| {
            tags.and_then(|t| t.get(k))
                .and_then(|v| v.as_str())
                .map(|s| s.to_owned())
        };
        let bounds = points.iter().fold(
            (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
            |b, p| (b.0.min(p.lat), b.1.min(p.lon), b.2.max(p.lat), b.3.max(p.lon)),
        );
        ways.push(Way {
            id,
            bounds,
            kind: WayKind::from_tag(tag("highway").as_deref().unwrap_or("")),
            name: tag("name"),
            nodes,
            points,
            sac_scale: tag("sac_scale"),
        });
    }
    Ok(ways)
}

/// Longueur d'une polyligne, en mètres.
pub fn path_length_m(points: &[LatLon]) -> f64 {
    points
        .windows(2)
        .map(|w| crate::geo::haversine_m(w[0], w[1]))
        .sum()
}

// ---------------------------------------------------------------------------
// Cache par zone
// ---------------------------------------------------------------------------

/// (zone, numéro de tentative, corps de la réponse)
type ZoneReply = (ZoneKey, usize, Result<String, String>);

/// Requêtes simultanées vers Overpass. `https://overpass-api.de/api/status`
/// annonce « Rate limit: 2 » : au-delà, les suivantes attendent côté serveur et
/// une petite zone urgente se retrouve coincée derrière un gros bassin.
const MAX_IN_FLIGHT: usize = 2;

/// Délai au-delà duquel on considère une requête perdue et on bascule d'instance.
/// Ni `fetch` (navigateur) ni `ehttp` n'exposent de timeout : sans ce garde-fou,
/// une instance qui ne répond jamais immobilise un créneau pour toute la session.
///
/// Cas vu en vrai le 03/09/2026 : `overpass-api.de` répond `200` en IPv4 et rien
/// du tout en IPv6 (connexion TLS réinitialisée) depuis la machine de dev. Le
/// navigateur tentant l'IPv6 en premier, sans ce délai l'application resterait
/// bloquée sur une instance morte.
const REQUEST_TIMEOUT_S: f64 = 12.0;

pub struct TrailStore {
    pub net: TrailNetwork,
    source: Rc<dyn TrailSource>,
    inbox: Arc<Mutex<Vec<ZoneReply>>>,
    loaded: HashSet<ZoneKey>,
    /// Zones demandées mais pas encore envoyées.
    queue: VecDeque<(ZoneKey, usize)>,
    in_flight: usize,
    /// Zone en vol → (numéro de tentative, instant d'envoi).
    pending: HashMap<ZoneKey, (usize, web_time::Instant)>,
    failed: HashMap<ZoneKey, String>,
    pub last_error: Option<String>,
}

impl TrailStore {
    pub fn new(source: Rc<dyn TrailSource>) -> Self {
        Self {
            net: TrailNetwork::default(),
            source,
            inbox: Arc::new(Mutex::new(Vec::new())),
            loaded: HashSet::new(),
            queue: VecDeque::new(),
            in_flight: 0,
            pending: HashMap::new(),
            failed: HashMap::new(),
            last_error: None,
        }
    }

    pub fn overpass() -> Self {
        Self::new(Rc::new(OverpassSource::default()))
    }

    /// Vrai si ce point est couvert par une zone déjà en mémoire, **quel que
    /// soit son niveau** : une grande zone d'isochrone couvre les zones fines
    /// qu'elle contient.
    pub fn zone_ready(&self, ll: LatLon) -> bool {
        (0..ZONE_LEVELS.len() as u8).any(|l| self.loaded.contains(&ZoneKey::of_level(ll, l)))
    }

    pub fn zone_failed(&self, ll: LatLon) -> Option<&str> {
        self.failed.get(&ZoneKey::of(ll)).map(|s| s.as_str())
    }

    /// Demande la zone contenant ce point si elle manque. Idempotent : c'est tout
    /// le cache — une zone chargée n'est jamais redemandée.
    pub fn ensure(&mut self, ll: LatLon, ctx: &egui::Context) {
        if self.zone_ready(ll) {
            return;
        }
        self.ensure_key(ZoneKey::of(ll), ctx);
    }

    fn ensure_key(&mut self, key: ZoneKey, ctx: &egui::Context) {
        if self.loaded.contains(&key)
            || self.pending.contains_key(&key)
            || self.queue.iter().any(|(k, _)| *k == key)
            || self.failed.contains_key(&key)
        {
            return;
        }
        self.spawn(key, 0, ctx);
    }

    /// Charge les grandes zones couvrant un disque — ce dont l'isochrone a besoin.
    /// Renvoie le nombre de zones demandées, plafonné : Overpass est un service
    /// public à quotas, et une isochrone de 8 h couvre déjà ~25 km de rayon.
    pub fn ensure_area(
        &mut self,
        center: LatLon,
        radius_m: f64,
        max_zones: usize,
        ctx: &egui::Context,
    ) -> usize {
        let level = (ZONE_LEVELS.len() - 1) as u8;
        let size = ZONE_LEVELS[level as usize];
        let dlat = radius_m / 111_132.0;
        let dlon = radius_m / (111_320.0 * center.lat.to_radians().cos()).max(1.0);

        let mut keys = Vec::new();
        let mut lat = ((center.lat - dlat) / size).floor() * size;
        while lat <= center.lat + dlat {
            let mut lon = ((center.lon - dlon) / size).floor() * size;
            while lon <= center.lon + dlon {
                keys.push(ZoneKey::of_level(
                    LatLon::new(lat + size / 2.0, lon + size / 2.0),
                    level,
                ));
                lon += size;
            }
            lat += size;
        }
        // Les plus proches du centre d'abord : si le plafond coupe, on garde
        // l'utile.
        keys.sort_by_key(|k| {
            let (s, w, n, e) = k.bbox();
            let c = LatLon::new((s + n) / 2.0, (w + e) / 2.0);
            crate::geo::haversine_m(c, center) as i64
        });
        let mut asked = 0;
        for key in keys.into_iter().take(max_zones) {
            if !self.loaded.contains(&key) && !self.pending.contains_key(&key) {
                asked += 1;
            }
            self.failed.remove(&key);
            self.ensure_key(key, ctx);
        }
        asked
    }

    /// Relance les zones en échec (bouton « réessayer »).
    pub fn retry_failed(&mut self, ctx: &egui::Context) {
        let zones: Vec<ZoneKey> = self.failed.keys().copied().collect();
        self.failed.clear();
        self.last_error = None;
        for key in zones {
            self.spawn(key, 0, ctx);
        }
    }

    fn spawn(&mut self, key: ZoneKey, attempt: usize, ctx: &egui::Context) {
        // Les petites zones (le clic de l'utilisateur) passent devant les gros
        // bassins d'isochrone : c'est l'interaction qui attend, pas l'inverse.
        if key.level == 0 {
            self.queue.push_front((key, attempt));
        } else {
            self.queue.push_back((key, attempt));
        }
        self.dispatch(ctx);
    }

    fn dispatch(&mut self, ctx: &egui::Context) {
        while self.in_flight < MAX_IN_FLIGHT {
            let Some((key, attempt)) = self.queue.pop_front() else {
                return;
            };
            self.send(key, attempt, ctx);
        }
    }

    fn send(&mut self, key: ZoneKey, attempt: usize, ctx: &egui::Context) {
        self.in_flight += 1;
        self.pending.insert(key, (attempt, web_time::Instant::now()));
        let inbox = Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        self.source.request(
            key,
            attempt,
            Box::new(move |body| {
                inbox.lock().unwrap().push((key, attempt, body));
                ctx.request_repaint();
            }),
        );
    }

    /// Renvoie vrai si de nouvelles zones sont entrées dans le réseau.
    pub fn pump(&mut self, ctx: &egui::Context) -> bool {
        let arrived = {
            let mut inbox = self.inbox.lock().unwrap();
            std::mem::take(&mut *inbox)
        };
        let mut changed = false;
        for (key, attempt, body) in arrived {
            // Réponse d'une tentative déjà abandonnée sur expiration : son créneau
            // a été rendu, ne pas le rendre deux fois.
            if self.pending.get(&key).map(|(a, _)| *a) != Some(attempt) {
                continue;
            }
            self.pending.remove(&key);
            self.in_flight = self.in_flight.saturating_sub(1);
            match body.and_then(|b| parse_overpass(&b)) {
                Ok(ways) => {
                    for way in ways {
                        self.net.insert(way);
                    }
                    self.loaded.insert(key);
                    changed = true;
                }
                Err(e) => {
                    // 504 « too busy » est le cas courant : on bascule d'instance
                    // avant d'abandonner.
                    if attempt + 1 < self.source.endpoint_count() {
                        log::warn!("Overpass {e} — nouvelle instance");
                        self.spawn(key, attempt + 1, ctx);
                    } else {
                        log::warn!("Overpass abandonne : {e}");
                        self.last_error = Some(e.clone());
                        self.failed.insert(key, e);
                    }
                }
            }
        }
        self.expire(ctx);
        self.dispatch(ctx);
        changed
    }

    /// Abandonne les requêtes trop vieilles et bascule d'instance.
    fn expire(&mut self, ctx: &egui::Context) {
        let stale: Vec<(ZoneKey, usize)> = self
            .pending
            .iter()
            .filter(|(_, (_, sent))| sent.elapsed().as_secs_f64() > REQUEST_TIMEOUT_S)
            .map(|(k, (a, _))| (*k, *a))
            .collect();
        for (key, attempt) in stale {
            self.pending.remove(&key);
            self.in_flight = self.in_flight.saturating_sub(1);
            let msg = format!("pas de réponse en {REQUEST_TIMEOUT_S:.0} s");
            if attempt + 1 < self.source.endpoint_count() {
                log::warn!("Overpass {msg} — nouvelle instance");
                self.spawn(key, attempt + 1, ctx);
            } else {
                self.last_error = Some(msg.clone());
                self.failed.insert(key, msg);
            }
        }
    }

    /// (zones chargées, en cours, en échec, chemins)
    pub fn stats(&self) -> (usize, usize, usize, usize) {
        (
            self.loaded.len(),
            self.pending.len() + self.queue.len(),
            self.failed.len(),
            self.net.len(),
        )
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    /// Deux chemins en L qui se touchent, autour de (45.900, 6.870).
    const SAMPLE: &str = r#"{"version":0.6,"elements":[
      {"type":"way","id":1,"nodes":[10,11,12],"tags":{"highway":"path","name":"Sentier du Test","sac_scale":"hiking"},
       "geometry":[{"lat":45.9000,"lon":6.8700},{"lat":45.9010,"lon":6.8700},{"lat":45.9010,"lon":6.8720}]},
      {"type":"way","id":2,"nodes":[12,13],"tags":{"highway":"track"},
       "geometry":[{"lat":45.9010,"lon":6.8720},{"lat":45.9030,"lon":6.8720}]},
      {"type":"node","id":10,"lat":45.9,"lon":6.87},
      {"type":"way","id":3,"tags":{"highway":"path"},"geometry":[{"lat":45.9,"lon":6.87}]}
    ]}"#;

    fn network() -> TrailNetwork {
        let mut net = TrailNetwork::default();
        for way in parse_overpass(SAMPLE).unwrap() {
            net.insert(way);
        }
        net
    }

    #[test]
    fn parse_ignore_noeuds_et_geometries_degenerees() {
        let ways = parse_overpass(SAMPLE).unwrap();
        assert_eq!(ways.len(), 2, "le nœud et le chemin à 1 point sont écartés");
        assert_eq!(ways[0].kind, WayKind::Path);
        assert_eq!(ways[0].name.as_deref(), Some("Sentier du Test"));
        assert_eq!(ways[0].nodes, vec![10, 11, 12]);
        assert_eq!(ways[1].kind, WayKind::Track);
    }

    #[test]
    fn erreur_html_signalee() {
        assert!(parse_overpass("<html>too busy</html>").is_err());
    }

    #[test]
    fn insertion_idempotente() {
        // Overpass renvoie les chemins entiers : deux zones voisines rapportent
        // les mêmes chemins de bordure, ils ne doivent pas être dupliqués.
        let mut net = network();
        for way in parse_overpass(SAMPLE).unwrap() {
            net.insert(way);
        }
        assert_eq!(net.len(), 2);
    }

    #[test]
    fn snap_trouve_le_segment_le_plus_proche() {
        let net = network();
        // ~15 m à l'est du premier segment (vertical, lon 6.8700).
        let click = LatLon::new(45.9005, 6.87019);
        let snap = net.snap(click, SNAP_RADIUS_M).expect("doit accrocher");
        assert_eq!(snap.way_id, 1);
        assert_eq!(snap.seg, 0);
        assert!(snap.dist_m < 20.0, "dist = {}", snap.dist_m);
        assert!((snap.pos.lon - 6.8700).abs() < 1e-6);
        assert!((snap.t - 0.5).abs() < 0.05, "t = {}", snap.t);
    }

    #[test]
    fn snap_refuse_au_dela_du_rayon() {
        let net = network();
        // ~800 m à l'ouest de tout.
        assert!(net.snap(LatLon::new(45.9005, 6.860), SNAP_RADIUS_M).is_none());
    }

    #[test]
    fn suivi_de_troncon_dans_les_deux_sens() {
        let net = network();
        let a = net.snap(LatLon::new(45.9002, 6.8700), SNAP_RADIUS_M).unwrap();
        let b = net.snap(LatLon::new(45.9010, 6.8715), SNAP_RADIUS_M).unwrap();
        let forward = net.follow(&a, &b).expect("même chemin OSM");
        // Doit passer par le coude (45.9010, 6.8700) et non couper en diagonale.
        assert!(forward.len() >= 3, "{forward:?}");
        assert!(forward
            .iter()
            .any(|p| (p.lat - 45.9010).abs() < 1e-9 && (p.lon - 6.8700).abs() < 1e-9));
        let backward = net.follow(&b, &a).unwrap();
        assert_eq!(backward.len(), forward.len());
        assert!((backward[0].lat - forward.last().unwrap().lat).abs() < 1e-9);
    }

    #[test]
    fn pas_de_suivi_entre_chemins_differents() {
        let net = network();
        let a = net.snap(LatLon::new(45.9002, 6.8700), SNAP_RADIUS_M).unwrap();
        let b = net.snap(LatLon::new(45.9025, 6.8720), SNAP_RADIUS_M).unwrap();
        assert_eq!(b.way_id, 2);
        // La jonction des deux chemins, c'est le graphe du M2.
        assert!(net.follow(&a, &b).is_none());
    }

    #[test]
    fn zone_contient_son_point() {
        let ll = LatLon::new(45.9234, 6.8712);
        let (s, w, n, e) = ZoneKey::of(ll).bbox();
        assert!(s <= ll.lat && ll.lat < n && w <= ll.lon && ll.lon < e);
        assert!((n - s - ZONE_DEG).abs() < 1e-12);
        // Deux points de la même zone partagent la clé : c'est ce qui évite de
        // redemander la zone.
        assert_eq!(ZoneKey::of(ll), ZoneKey::of(LatLon::new(45.9236, 6.8714)));
    }

    #[test]
    fn requete_overpass_bien_formee() {
        let q = OverpassSource::query(ZoneKey::of(LatLon::new(45.92, 6.87)));
        assert!(q.contains("[out:json]"), "{q}");
        assert!(q.contains("out geom;"), "{q}");
        assert!(q.contains("(45.9200,6.8600,45.9400,6.8800)"), "{q}");
    }

    /// Récupère une zone en bloquant, en basculant d'instance au besoin.
    /// Overpass répond régulièrement 504 « too busy ».
    #[cfg(not(target_arch = "wasm32"))]
    pub fn fetch_zone_blocking(zone: ZoneKey) -> Result<Vec<Way>, String> {
        let src = OverpassSource::default();
        let mut last_err = String::new();
        src.endpoints
            .iter()
            .find_map(|url| {
                let mut request =
                    ehttp::Request::post(url.clone(), OverpassSource::query(zone).into_bytes());
                request
                    .headers
                    .insert("Content-Type", "text/plain;charset=UTF-8");
                match ehttp::fetch_blocking(&request) {
                    Ok(resp) if resp.ok => parse_overpass(resp.text().unwrap_or_default()).ok(),
                    Ok(resp) => {
                        last_err = format!("{url} → HTTP {}", resp.status);
                        None
                    }
                    Err(e) => {
                        last_err = format!("{url} → {e}");
                        None
                    }
                }
            })
            .ok_or_else(|| format!("aucune instance Overpass n'a répondu : {last_err}"))
    }

    /// Bout en bout contre Overpass : `cargo test -- --ignored`.
    #[test]
    #[ignore = "réseau"]
    #[cfg(not(target_arch = "wasm32"))]
    fn zone_reelle_chamonix() {
        let zone = ZoneKey::of(LatLon::new(45.92, 6.87));
        let ways = fetch_zone_blocking(zone).unwrap();
        assert!(ways.len() > 50, "{} chemins", ways.len());

        let mut net = TrailNetwork::default();
        for way in ways {
            net.insert(way);
        }
        // La gare de Chamonix est à quelques mètres d'une voie piétonne.
        let snap = net.snap(LatLon::new(45.9237, 6.8703), SNAP_RADIUS_M);
        assert!(snap.is_some(), "aucun sentier près du centre de Chamonix");
    }
}
