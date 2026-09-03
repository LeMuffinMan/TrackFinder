//! Tracé : polyligne, échantillonnage, profil altimétrique, temps de marche.

use crate::dem::DemStore;
use crate::geo::{haversine_m, lerp_latlon, LatLon};

/// Pas d'échantillonnage du profil. 50 m est plus fin que le MNT (~5 m/px au
/// zoom 14) sans faire exploser le nombre de tuiles demandées.
const STEP_M: f64 = 50.0;
const MAX_SAMPLES: usize = 4000;
/// Hystérésis sur le dénivelé : sans ça le bruit du MNT gonfle le D+.
const ASCENT_THRESHOLD_M: f32 = 3.0;

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub dist_m: f64,
    pub pos: LatLon,
    pub elev_m: Option<f32>,
}

#[derive(Clone, Copy, Debug)]
pub struct WalkSettings {
    /// Vitesse sur le plat, km/h (base Naismith : 5).
    pub flat_kmh: f64,
    /// Vitesse d'ascension, m/h (base Naismith : 600).
    pub ascent_mh: f64,
    pub body_weight_kg: f64,
    pub pack_weight_kg: f64,
}

impl Default for WalkSettings {
    fn default() -> Self {
        Self {
            flat_kmh: 5.0,
            ascent_mh: 600.0,
            body_weight_kg: 70.0,
            pack_weight_kg: 12.0,
        }
    }
}

impl WalkSettings {
    /// facteur_vitesse = 1 − 0.01 × max(0, poids_sac − 10)
    pub fn speed_factor(&self) -> f64 {
        1.0 - 0.01 * (self.pack_weight_kg - 10.0).max(0.0)
    }

    /// seuil_charge = poids_corporel × 0.20
    pub fn load_limit_kg(&self) -> f64 {
        self.body_weight_kg * 0.20
    }

    pub fn overloaded(&self) -> bool {
        self.pack_weight_kg > self.load_limit_kg()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TrackStats {
    pub distance_m: f64,
    pub ascent_m: f64,
    pub descent_m: f64,
    pub min_elev_m: Option<f32>,
    pub max_elev_m: Option<f32>,
    pub time_h: f64,
    /// Faux tant que des tuiles MNT manquent : les chiffres sont partiels.
    pub elevation_complete: bool,
}

#[derive(Default)]
pub struct Track {
    pub points: Vec<LatLon>,
    profile: Vec<Sample>,
    stats: TrackStats,
    dirty: bool,
}

impl Track {
    pub fn push(&mut self, ll: LatLon) {
        self.points.push(ll);
        self.dirty = true;
    }

    pub fn pop(&mut self) {
        self.points.pop();
        self.dirty = true;
    }

    pub fn clear(&mut self) {
        self.points.clear();
        self.dirty = true;
    }

    pub fn profile(&self) -> &[Sample] {
        &self.profile
    }

    pub fn stats(&self) -> &TrackStats {
        &self.stats
    }

    /// Recalcule si nécessaire. Tant que le MNT n'est pas complet on recalcule à
    /// chaque frame : les tuiles arrivent au fil de l'eau et complètent le profil.
    pub fn refresh(&mut self, dem: &mut DemStore, settings: &WalkSettings, ctx: &egui::Context) {
        if !self.dirty && self.stats.elevation_complete {
            return;
        }
        self.dirty = false;
        self.profile = sample_polyline(&self.points);
        let mut complete = true;
        for s in &mut self.profile {
            s.elev_m = dem.elevation(s.pos, ctx);
            complete &= s.elev_m.is_some();
        }
        self.stats = compute_stats(&self.profile, settings, complete);
    }

    /// Recalcule seulement le temps (changement de réglage, sans retoucher au MNT).
    pub fn recompute_time(&mut self, settings: &WalkSettings) {
        self.stats = compute_stats(&self.profile, settings, self.stats.elevation_complete);
    }
}

/// Découpe la polyligne en points espacés d'environ `STEP_M`, sommets compris.
fn sample_polyline(points: &[LatLon]) -> Vec<Sample> {
    let mut out = Vec::new();
    if points.is_empty() {
        return out;
    }
    let total: f64 = points.windows(2).map(|w| haversine_m(w[0], w[1])).sum();
    let step = (total / MAX_SAMPLES as f64).max(STEP_M);

    let mut dist = 0.0;
    out.push(Sample {
        dist_m: 0.0,
        pos: points[0],
        elev_m: None,
    });
    for w in points.windows(2) {
        let seg = haversine_m(w[0], w[1]);
        if seg <= 0.0 {
            continue;
        }
        let n = (seg / step).ceil().max(1.0) as usize;
        for i in 1..=n {
            let t = i as f64 / n as f64;
            out.push(Sample {
                dist_m: dist + seg * t,
                pos: lerp_latlon(w[0], w[1], t),
                elev_m: None,
            });
        }
        dist += seg;
    }
    out
}

fn compute_stats(profile: &[Sample], settings: &WalkSettings, complete: bool) -> TrackStats {
    let mut stats = TrackStats {
        distance_m: profile.last().map(|s| s.dist_m).unwrap_or(0.0),
        elevation_complete: complete && !profile.is_empty(),
        ..Default::default()
    };

    // Dénivelés avec hystérésis : on n'enregistre un changement de sens qu'au-delà
    // du seuil, ce qui écarte le bruit du MNT.
    let mut anchor: Option<f32> = None;
    for s in profile.iter().filter_map(|s| s.elev_m) {
        stats.min_elev_m = Some(stats.min_elev_m.map_or(s, |m: f32| m.min(s)));
        stats.max_elev_m = Some(stats.max_elev_m.map_or(s, |m: f32| m.max(s)));
        match anchor {
            None => anchor = Some(s),
            Some(a) => {
                let d = s - a;
                if d > ASCENT_THRESHOLD_M {
                    stats.ascent_m += d as f64;
                    anchor = Some(s);
                } else if d < -ASCENT_THRESHOLD_M {
                    stats.descent_m += (-d) as f64;
                    anchor = Some(s);
                }
            }
        }
    }

    // Naismith : plat + ascension, corrigé du facteur de charge.
    let factor = settings.speed_factor().max(0.1);
    let flat_h = (stats.distance_m / 1000.0) / settings.flat_kmh.max(0.1);
    let up_h = stats.ascent_m / settings.ascent_mh.max(1.0);
    stats.time_h = (flat_h + up_h) / factor;
    stats
}

pub fn format_duration(hours: f64) -> String {
    if !hours.is_finite() || hours <= 0.0 {
        return "—".to_owned();
    }
    let total_min = (hours * 60.0).round() as i64;
    format!("{} h {:02}", total_min / 60, total_min % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(elevs: &[f32], total_m: f64) -> Vec<Sample> {
        let n = elevs.len().max(2) - 1;
        elevs
            .iter()
            .enumerate()
            .map(|(i, e)| Sample {
                dist_m: total_m * i as f64 / n as f64,
                pos: LatLon::new(45.0, 6.0),
                elev_m: Some(*e),
            })
            .collect()
    }

    #[test]
    fn charge_penalise_la_vitesse() {
        let s = WalkSettings {
            pack_weight_kg: 20.0,
            ..Default::default()
        };
        assert!((s.speed_factor() - 0.90).abs() < 1e-9);
        // Sous 10 kg, aucune pénalité.
        let light = WalkSettings {
            pack_weight_kg: 8.0,
            ..Default::default()
        };
        assert!((light.speed_factor() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn seuil_de_charge_a_20_pourcent() {
        let s = WalkSettings {
            body_weight_kg: 70.0,
            pack_weight_kg: 15.0,
            ..Default::default()
        };
        assert!((s.load_limit_kg() - 14.0).abs() < 1e-9);
        assert!(s.overloaded());
    }

    #[test]
    fn naismith_plat() {
        let s = WalkSettings {
            pack_weight_kg: 0.0,
            ..Default::default()
        };
        let stats = compute_stats(&samples(&[1000.0, 1000.0], 10_000.0), &s, true);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
        assert_eq!(stats.ascent_m, 0.0);
    }

    #[test]
    fn naismith_ascension() {
        let s = WalkSettings {
            pack_weight_kg: 0.0,
            ..Default::default()
        };
        // 5 km plat (1 h) + 600 m de montée (1 h)
        let stats = compute_stats(&samples(&[1000.0, 1600.0], 5_000.0), &s, true);
        assert!((stats.ascent_m - 600.0).abs() < 1e-6);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
    }

    #[test]
    fn hysteresis_filtre_le_bruit_du_mnt() {
        let s = WalkSettings::default();
        // Oscillations de ±1 m : du bruit, pas du dénivelé.
        let noisy: Vec<f32> = (0..100)
            .map(|i| 1000.0 + if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let stats = compute_stats(&samples(&noisy, 1000.0), &s, true);
        assert_eq!(stats.ascent_m, 0.0);
        assert_eq!(stats.descent_m, 0.0);

        // Une vraie montée de 100 m est bien comptée.
        let real = compute_stats(&samples(&[1000.0, 1050.0, 1100.0], 1000.0), &s, true);
        assert!((real.ascent_m - 100.0).abs() < 1e-6);
    }

    #[test]
    fn echantillonnage_respecte_le_pas() {
        // ~1.11 km nord-sud
        let pts = vec![LatLon::new(45.0, 6.0), LatLon::new(45.01, 6.0)];
        let out = sample_polyline(&pts);
        assert!(out.len() > 20, "{} échantillons", out.len());
        for w in out.windows(2) {
            assert!(w[1].dist_m - w[0].dist_m <= STEP_M + 1.0);
        }
        assert!((out.last().unwrap().dist_m - haversine_m(pts[0], pts[1])).abs() < 1.0);
    }

    #[test]
    fn echantillonnage_plafonne() {
        // 100 km : le pas s'élargit au lieu de faire exploser le nombre de points.
        let pts = vec![LatLon::new(45.0, 6.0), LatLon::new(45.9, 6.0)];
        let out = sample_polyline(&pts);
        assert!(out.len() <= MAX_SAMPLES + 2, "{} échantillons", out.len());
    }

    #[test]
    fn duree_formatee() {
        assert_eq!(format_duration(2.5), "2 h 30");
        assert_eq!(format_duration(0.0), "—");
    }
}
