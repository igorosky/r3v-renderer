pub const HYPSOMETRIC_COLORMAP_LEN: usize = 256;

const STOPS: [(f32, [f32; 3]); 10] = [
    (0.00, [0.05, 0.07, 0.30]),
    (0.20, [0.07, 0.25, 0.60]),
    (0.38, [0.16, 0.55, 0.78]),
    (0.47, [0.52, 0.82, 0.80]),
    (0.50, [0.72, 0.87, 0.60]),
    (0.56, [0.36, 0.70, 0.36]),
    (0.66, [0.80, 0.78, 0.38]),
    (0.78, [0.72, 0.48, 0.24]),
    (0.90, [0.50, 0.34, 0.26]),
    (1.00, [0.97, 0.97, 0.97]),
];

fn color_at(position: f32) -> [f32; 3] {
    let upper = STOPS
        .iter()
        .position(|&(stop, _)| position <= stop)
        .unwrap_or(STOPS.len() - 1)
        .max(1);
    let (start, from) = STOPS[upper - 1];
    let (end, to) = STOPS[upper];
    let t = (position - start) / (end - start);
    std::array::from_fn(|k| from[k] + (to[k] - from[k]) * t)
}

pub fn hypsometric_colormap() -> impl Iterator<Item = [f32; 3]> {
    (0..HYPSOMETRIC_COLORMAP_LEN)
        .map(|index| color_at(index as f32 / (HYPSOMETRIC_COLORMAP_LEN - 1) as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stops_cover_the_colormap() {
        assert_eq!(STOPS[0].0, 0.0);
        assert_eq!(STOPS[STOPS.len() - 1].0, 1.0);
        assert!(STOPS.windows(2).all(|pair| pair[0].0 < pair[1].0));
    }

    #[test]
    fn colors_pass_through_the_stops() {
        for (position, color) in STOPS {
            let sampled = color_at(position);
            for k in 0..3 {
                assert!(
                    (sampled[k] - color[k]).abs() < 1e-5,
                    "{position}: {sampled:?}"
                );
            }
        }
    }

    #[test]
    fn colors_stay_between_their_stops() {
        let colors = hypsometric_colormap().collect::<Vec<_>>();
        assert_eq!(colors.len(), HYPSOMETRIC_COLORMAP_LEN);
        assert!(colors.iter().flatten().all(|c| (0.0..=1.0).contains(c)));
    }

    #[test]
    fn depths_are_blue_and_heights_are_not() {
        let [r, _, b] = color_at(0.25);
        assert!(b > r);
        let [r, _, b] = color_at(0.75);
        assert!(r > b);
    }
}
