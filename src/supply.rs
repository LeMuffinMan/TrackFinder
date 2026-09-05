//! Food and water carried, leg by leg.
//!
//! The pack is not a constant: it is the gear plus what is left of the
//! consumables. Both get lighter every day and jump back up at a resupply, and
//! since the load drives the speed factor, the walking time of a leg has to be
//! priced with the load carried *on that leg*, not with a single figure for the
//! whole trip.
//!
//! That ordering works out because the dependency only ever runs forward: a
//! leg's start load depends on the legs before it, never on itself. One pass,
//! no fixed point.

use crate::track::{TrackStats, WalkSettings};

/// Water weighs a kilogram per litre. Named rather than inlined, because it is
/// the one place the two units meet.
pub const WATER_KG_PER_L: f64 = 1.0;

#[derive(Clone, Copy, Debug)]
pub struct SupplySettings {
    /// Gear, consumables excluded — what the README calls the empty pack.
    pub base_pack_kg: f64,
    pub ration_g_per_day: f64,
    /// What the bottles and bladder hold, all together.
    pub water_capacity_l: f64,
    /// Drinking rate per hour of walking. Per hour and not per kilometre: the
    /// walking time already carries both the distance and the climb, so one
    /// number does the work of two.
    pub water_rate_lph: f64,
    /// Food in the pack on day one.
    pub food_start_kg: f64,
}

impl Default for SupplySettings {
    fn default() -> Self {
        Self {
            base_pack_kg: 8.0,
            ration_g_per_day: 700.0,
            water_capacity_l: 2.0,
            water_rate_lph: 0.5,
            food_start_kg: 2.1,
        }
    }
}

impl SupplySettings {
    pub fn ration_kg(&self) -> f64 {
        self.ration_g_per_day / 1000.0
    }
}

/// What the user takes on at a waypoint, before setting off again. Manual on
/// purpose: village and water-point detection is M6, and a resupply the user
/// did not plan is not a resupply.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Resupply {
    pub food_kg: f64,
    /// Fill the containers back to capacity here.
    pub water_fill: bool,
}

impl Resupply {
    /// The start of the trip: bottles full, food already counted in
    /// `food_start_kg`.
    pub fn at_start() -> Self {
        Self {
            food_kg: 0.0,
            water_fill: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LegSupply {
    pub food_start_kg: f64,
    pub food_end_kg: f64,
    pub water_start_l: f64,
    pub water_end_l: f64,
    /// What the leg asks for, which can exceed what is carried.
    pub water_need_l: f64,
    pub pack_start_kg: f64,
    /// Time for this leg at its own load — not the one shown for the whole track.
    pub time_h: f64,
    /// Pack above the recommended threshold when setting off. Evaluated per leg
    /// because the peak lands just after a resupply, not necessarily on day one.
    pub overloaded: bool,
    pub food_short: bool,
    pub water_short: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Plan {
    pub legs: Vec<LegSupply>,
    /// Load on the back at the last waypoint, resupply there included: what the
    /// next leg — the one the isochrone is about to plan — would start with.
    pub next_pack_kg: f64,
    pub next_food_kg: f64,
    pub next_water_l: f64,
}

impl Plan {
    pub fn total_time_h(&self) -> f64 {
        self.legs.iter().map(|l| l.time_h).sum()
    }

    pub fn any_overloaded(&self) -> bool {
        self.legs.iter().any(|l| l.overloaded)
    }

    pub fn first_food_short(&self) -> Option<usize> {
        self.legs.iter().position(|l| l.food_short)
    }

    pub fn first_water_short(&self) -> Option<usize> {
        self.legs.iter().position(|l| l.water_short)
    }
}

/// Walks the legs in order, spending and topping up.
///
/// `resupply` is indexed by waypoint: entry `i` is taken on at the start of leg
/// `i`, and the entry for the last waypoint feeds `next_*`. Missing entries read
/// as "nothing taken on".
pub fn simulate(
    legs: &[TrackStats],
    resupply: &[Resupply],
    walk: &WalkSettings,
    sup: &SupplySettings,
) -> Plan {
    let mut food = sup.food_start_kg.max(0.0);
    let mut water = sup.water_capacity_l.max(0.0);
    let ration = sup.ration_kg().max(0.0);
    let limit = walk.load_limit_kg();

    let mut plan = Plan::default();
    for (i, leg) in legs.iter().enumerate() {
        let r = resupply.get(i).copied().unwrap_or_default();
        food += r.food_kg.max(0.0);
        if r.water_fill {
            water = sup.water_capacity_l.max(0.0);
        }

        let food_start = food;
        let water_start = water;
        let pack_start = sup.base_pack_kg + food_start + water_start * WATER_KG_PER_L;

        // The load is taken at the start of the leg and held for its whole
        // length. Slightly pessimistic — the pack does lighten as the day goes
        // on — and pessimistic is the right way to be wrong about a pack.
        let time_h = leg.base_time_h / walk.speed_factor_for(pack_start).max(0.1);

        let water_need = sup.water_rate_lph.max(0.0) * time_h;
        let water_short = water_need > water_start + 1e-9;
        water = (water_start - water_need).max(0.0);

        let food_short = ration > food_start + 1e-9;
        food = (food_start - ration).max(0.0);

        plan.legs.push(LegSupply {
            food_start_kg: food_start,
            food_end_kg: food,
            water_start_l: water_start,
            water_end_l: water,
            water_need_l: water_need,
            pack_start_kg: pack_start,
            time_h,
            overloaded: pack_start > limit,
            food_short,
            water_short,
        });
    }

    // State at the last waypoint, where the next leg will start.
    let r = resupply.get(legs.len()).copied().unwrap_or_default();
    plan.next_food_kg = food + r.food_kg.max(0.0);
    plan.next_water_l = if r.water_fill {
        sup.water_capacity_l.max(0.0)
    } else {
        water
    };
    plan.next_pack_kg =
        sup.base_pack_kg + plan.next_food_kg + plan.next_water_l * WATER_KG_PER_L;
    plan
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leg(base_time_h: f64) -> TrackStats {
        TrackStats {
            base_time_h,
            ..Default::default()
        }
    }

    fn settings() -> (WalkSettings, SupplySettings) {
        let walk = WalkSettings {
            body_weight_kg: 70.0,
            ..Default::default()
        };
        let sup = SupplySettings {
            base_pack_kg: 8.0,
            ration_g_per_day: 700.0,
            water_capacity_l: 2.0,
            water_rate_lph: 0.5,
            food_start_kg: 2.1,
        };
        (walk, sup)
    }

    #[test]
    fn the_pack_gets_lighter_every_day() {
        let (walk, sup) = settings();
        let legs = [leg(4.0), leg(4.0), leg(4.0)];
        let plan = simulate(&legs, &[], &walk, &sup);
        assert_eq!(plan.legs.len(), 3);
        // 8 + 2.1 food + 2 L water
        assert!((plan.legs[0].pack_start_kg - 12.1).abs() < 1e-9);
        for w in plan.legs.windows(2) {
            assert!(w[1].pack_start_kg < w[0].pack_start_kg);
        }
        // Three days of food eaten, none left over.
        assert!(plan.next_food_kg.abs() < 1e-9);
    }

    #[test]
    fn a_resupply_raises_the_load_mid_trip() {
        let (walk, sup) = settings();
        let legs = [leg(4.0), leg(4.0), leg(4.0)];
        let resupply = [
            Resupply::at_start(),
            Resupply::default(),
            Resupply {
                food_kg: 5.0,
                water_fill: true,
            },
        ];
        let plan = simulate(&legs, &resupply, &walk, &sup);
        // The peak is on leg 3, not on day one — the point of evaluating the
        // load per leg rather than once.
        assert!(plan.legs[2].pack_start_kg > plan.legs[0].pack_start_kg);
    }

    #[test]
    fn a_heavier_pack_makes_the_same_leg_longer() {
        let (walk, mut sup) = settings();
        let legs = [leg(4.0)];
        let light = simulate(&legs, &[], &walk, &sup);
        sup.food_start_kg = 12.0;
        let heavy = simulate(&legs, &[], &walk, &sup);
        assert!(heavy.legs[0].time_h > light.legs[0].time_h);
    }

    #[test]
    fn running_out_of_water_is_flagged_not_hidden() {
        let (walk, sup) = settings();
        // 2 L capacity, 0.5 L/h: anything past 4 h is short.
        let plan = simulate(&[leg(6.0)], &[], &walk, &sup);
        assert!(plan.legs[0].water_short);
        assert!(plan.legs[0].water_need_l > 2.0);
        // The tank floors at zero rather than going negative.
        assert_eq!(plan.legs[0].water_end_l, 0.0);
    }

    #[test]
    fn a_fill_restores_capacity_between_legs() {
        let (walk, sup) = settings();
        let legs = [leg(3.0), leg(3.0)];
        let no_fill = simulate(&legs, &[Resupply::at_start(), Resupply::default()], &walk, &sup);
        assert!(no_fill.legs[1].water_start_l < 2.0);
        assert!(no_fill.legs[1].water_short);

        let filled = simulate(&legs, &[Resupply::at_start(), Resupply::at_start()], &walk, &sup);
        assert!((filled.legs[1].water_start_l - 2.0).abs() < 1e-9);
        assert!(!filled.legs[1].water_short);
    }

    #[test]
    fn food_runs_out_on_the_day_after_the_last_ration() {
        let (walk, mut sup) = settings();
        sup.food_start_kg = 1.4; // two days
        let plan = simulate(&[leg(2.0), leg(2.0), leg(2.0)], &[], &walk, &sup);
        assert_eq!(plan.first_food_short(), Some(2));
    }

    #[test]
    fn the_overload_alert_is_per_leg() {
        let (walk, mut sup) = settings();
        // Limit is 14 kg. Start under it, resupply over it.
        sup.food_start_kg = 1.0;
        let legs = [leg(3.0), leg(3.0)];
        let resupply = [Resupply::at_start(), Resupply { food_kg: 6.0, water_fill: true }];
        let plan = simulate(&legs, &resupply, &walk, &sup);
        assert!(!plan.legs[0].overloaded);
        assert!(plan.legs[1].overloaded);
        assert!(plan.any_overloaded());
    }

    #[test]
    fn the_next_leg_starts_where_the_last_one_stopped() {
        let (walk, sup) = settings();
        let plan = simulate(&[leg(3.0)], &[], &walk, &sup);
        assert!((plan.next_food_kg - (2.1 - 0.7)).abs() < 1e-9);
        assert!((plan.next_pack_kg - (8.0 + plan.next_food_kg + plan.next_water_l)).abs() < 1e-9);
    }

    #[test]
    fn an_empty_track_still_gives_the_pack_at_the_start() {
        let (walk, sup) = settings();
        let plan = simulate(&[], &[], &walk, &sup);
        assert!(plan.legs.is_empty());
        assert!((plan.next_pack_kg - 12.1).abs() < 1e-9);
    }
}
