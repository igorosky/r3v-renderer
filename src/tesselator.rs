use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::VecDeque;

#[cfg(feature = "multi-threaded")]
use rayon::prelude::*;

use crate::{
    Tesselator,
    geometry::{
        Axis, DiscretePoint, DiscreteQuad, DiscreteUnit, Point, PointI, RealTriangle, RealUnit,
        TriangleChunk, Vector2D, Vector3D, VoxelNormal, VoxelQuad,
    },
};

const SUBCHUNK_SIZE: Vector3D<usize> = Vector3D { x: 4, y: 4, z: 2 }; // corresponds to input geometry delta `PointI` bitfield
const CHUNK_WIDTH: usize = 16; // in voxels
const CHUNK_MASK: u32 = u32::MAX >> (32 - CHUNK_WIDTH);
const CHUNK_SUBCHUNKS: Vector3D<usize> = Vector3D {
    x: CHUNK_WIDTH / SUBCHUNK_SIZE.x,
    y: CHUNK_WIDTH / SUBCHUNK_SIZE.y,
    z: CHUNK_WIDTH / SUBCHUNK_SIZE.z,
};
const CHUNK_RATIO: usize = CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y * CHUNK_SUBCHUNKS.z;
const MC_GRID_SIZE: usize = CHUNK_WIDTH + 1;
const MC_PADDED_GRID_SIZE: usize = CHUNK_WIDTH + 2;
const MC_CUBE_CORNERS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];
const MC_CUBE_TETRAHEDRA: [[usize; 4]; 6] = [
    [0, 5, 1, 6],
    [0, 1, 2, 6],
    [0, 2, 3, 6],
    [0, 3, 7, 6],
    [0, 7, 4, 6],
    [0, 4, 5, 6],
];
const MC_ISO_LEVEL: RealUnit = 0.5;

type Columns<T> = [[[T; CHUNK_WIDTH]; CHUNK_WIDTH]; 3];
type SolidColumns = Columns<u64>;

const fn layer_mask(axis: Axis, layer: usize) -> u32 {
    let mut mask = 0;
    let mut bit = 0;
    while bit < u32::BITS {
        if PointI::voxel_local(bit).get(axis) == layer {
            mask |= 1 << bit;
        }
        bit += 1;
    }
    mask
}

const BOUNDARY_LAYERS: [[u32; 2]; 3] = [
    [
        layer_mask(Axis::X, 0),
        layer_mask(Axis::X, SUBCHUNK_SIZE.get(Axis::X) - 1),
    ],
    [
        layer_mask(Axis::Y, 0),
        layer_mask(Axis::Y, SUBCHUNK_SIZE.get(Axis::Y) - 1),
    ],
    [
        layer_mask(Axis::Z, 0),
        layer_mask(Axis::Z, SUBCHUNK_SIZE.get(Axis::Z) - 1),
    ],
];

struct Chunk {
    subchunks: [u32; CHUNK_RATIO],
    columns: Columns<u32>,
    solid_subchunks: usize,
}

impl Chunk {
    fn new() -> Box<Self> {
        Box::new(Self {
            subchunks: [0; CHUNK_RATIO],
            columns: [[[0; CHUNK_WIDTH]; CHUNK_WIDTH]; 3],
            solid_subchunks: 0,
        })
    }

    fn subchunk_index(subchunk: Point<usize>) -> usize {
        (subchunk.z * CHUNK_SUBCHUNKS.y + subchunk.y) * CHUNK_SUBCHUNKS.x + subchunk.x
    }

    fn origin_of(point: DiscretePoint) -> DiscretePoint {
        point.map(|c| c.div_euclid(CHUNK_WIDTH as DiscreteUnit) * CHUNK_WIDTH as DiscreteUnit)
    }

    fn neighbour_of(chunk: DiscretePoint, axis: Axis, direction: DiscreteUnit) -> DiscretePoint {
        let mut neighbour = chunk;
        neighbour[axis] += direction * CHUNK_WIDTH as DiscreteUnit;
        neighbour
    }

    fn subchunk_mask(&self, subchunk: Point<usize>) -> u32 {
        self.subchunks[Self::subchunk_index(subchunk)]
    }

    fn voxel(&self, local: Point<usize>) -> bool {
        let subchunk = Point::from_fn(|axis| local[axis] / SUBCHUNK_SIZE[axis]);
        let bit = PointI::voxel_bit(Point::from_fn(|axis| local[axis] % SUBCHUNK_SIZE[axis]));
        self.subchunk_mask(subchunk) & (1 << bit) != 0
    }

    fn has_voxel_on_boundary(&self, boundary: [i8; 3]) -> bool {
        for z in 0..CHUNK_WIDTH {
            if boundary[2] != 0 && z != if boundary[2] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                continue;
            }
            for y in 0..CHUNK_WIDTH {
                if boundary[1] != 0 && y != if boundary[1] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                    continue;
                }
                for x in 0..CHUNK_WIDTH {
                    if boundary[0] != 0 && x != if boundary[0] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                        continue;
                    }
                    if self.voxel(Point { x, y, z }) {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn set_subchunk_mask(&mut self, subchunk: Point<usize>, mask: u32) {
        let previous = std::mem::replace(&mut self.subchunks[Self::subchunk_index(subchunk)], mask);
        self.solid_subchunks =
            self.solid_subchunks + (mask != 0) as usize - (previous != 0) as usize;
        let origin = Point::from_fn(|axis| subchunk[axis] * SUBCHUNK_SIZE[axis]);
        for normal in Axis::ALL {
            let (column, row) = normal.plane();
            let run = (1u32 << SUBCHUNK_SIZE[normal]) - 1;
            for r in 0..SUBCHUNK_SIZE[row] {
                for c in 0..SUBCHUNK_SIZE[column] {
                    let mut local = Point { x: 0, y: 0, z: 0 };
                    local[column] = c;
                    local[row] = r;
                    let mut bits = 0;
                    for n in 0..SUBCHUNK_SIZE[normal] {
                        local[normal] = n;
                        bits |= (mask >> PointI::voxel_bit(local) & 1) << n;
                    }
                    let voxels =
                        &mut self.columns[normal as usize][origin[row] + r][origin[column] + c];
                    *voxels = *voxels & !(run << origin[normal]) | bits << origin[normal];
                }
            }
        }
    }
}

pub struct GreedyMeshTesselatorConfiguration {
    pub voxel_size: u16,
}

pub struct GreedyMeshTesselator {
    config: Option<GreedyMeshTesselatorConfiguration>,
    chunks: FxHashMap<DiscretePoint, Box<Chunk>>,
    dirty_queue: VecDeque<DiscretePoint>,
    in_queue: FxHashSet<DiscretePoint>,
    marching_cubes_enabled: bool,
}

impl Default for GreedyMeshTesselator {
    fn default() -> Self {
        Self::new()
    }
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            config: None,
            chunks: FxHashMap::default(),
            dirty_queue: VecDeque::new(),
            in_queue: FxHashSet::default(),
            marching_cubes_enabled: false,
        }
    }

    pub fn marching_cubes_enabled(&self) -> bool {
        self.marching_cubes_enabled
    }

    pub fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        if self.marching_cubes_enabled == enabled {
            return;
        }
        self.marching_cubes_enabled = enabled;
        let mut dirty = Vec::new();
        for (&chunk, stored) in &self.chunks {
            dirty.push(chunk);
            for z in -1..=1 {
                for y in -1..=1 {
                    for x in -1..=1 {
                        if x == 0 && y == 0 && z == 0 {
                            continue;
                        }
                        let boundary = [
                            if x < 0 {
                                -1
                            } else if x > 0 {
                                1
                            } else {
                                0
                            },
                            if y < 0 {
                                -1
                            } else if y > 0 {
                                1
                            } else {
                                0
                            },
                            if z < 0 {
                                -1
                            } else if z > 0 {
                                1
                            } else {
                                0
                            },
                        ];
                        if !stored.has_voxel_on_boundary(boundary) {
                            continue;
                        }
                        dirty.push(DiscretePoint {
                            x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                            y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                            z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                        });
                    }
                }
            }
        }
        for chunk in dirty {
            self.mark_dirty(chunk);
        }
    }

    fn mark_dirty(&mut self, chunk: DiscretePoint) {
        if self.in_queue.insert(chunk) {
            self.dirty_queue.push_back(chunk);
        }
    }

    fn mark_subchunk_dirty(&mut self, chunk: DiscretePoint, subchunk: Point<usize>, changed: u32) {
        self.mark_dirty(chunk);
        let mut directions = [[0; 3]; 3];
        let mut counts = [1; 3];
        for axis in Axis::ALL {
            let [low_layer, high_layer] = BOUNDARY_LAYERS[axis as usize];
            if subchunk[axis] == 0 && changed & low_layer != 0 {
                directions[axis as usize][counts[axis as usize]] = -1;
                counts[axis as usize] += 1;
            }
            if subchunk[axis] == CHUNK_SUBCHUNKS[axis] - 1 && changed & high_layer != 0 {
                directions[axis as usize][counts[axis as usize]] = 1;
                counts[axis as usize] += 1;
            }
        }
        for &z in &directions[2][..counts[2]] {
            for &y in &directions[1][..counts[1]] {
                for &x in &directions[0][..counts[0]] {
                    if x == 0 && y == 0 && z == 0 {
                        continue;
                    }
                    let changed_axes = (x != 0) as u8 + (y != 0) as u8 + (z != 0) as u8;
                    if self.marching_cubes_enabled || changed_axes == 1 {
                        self.mark_dirty(DiscretePoint {
                            x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                            y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                            z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                        });
                    }
                }
            }
        }
    }

    fn solid_columns(&self, chunk: DiscretePoint) -> Box<SolidColumns> {
        let mut solid = Box::new([[[0u64; CHUNK_WIDTH]; CHUNK_WIDTH]; 3]);
        let own = self.chunks.get(&chunk);
        for normal in Axis::ALL {
            let below = self.chunks.get(&Chunk::neighbour_of(chunk, normal, -1));
            let above = self.chunks.get(&Chunk::neighbour_of(chunk, normal, 1));
            let normal = normal as usize;
            for r in 0..CHUNK_WIDTH {
                for c in 0..CHUNK_WIDTH {
                    let column = |chunk: Option<&Box<Chunk>>| {
                        chunk.map_or(0, |chunk| chunk.columns[normal][r][c] as u64)
                    };
                    solid[normal][r][c] = column(own) << 1
                        | column(below) >> (CHUNK_WIDTH - 1) & 1
                        | (column(above) & 1) << (CHUNK_WIDTH + 1);
                }
            }
        }
        solid
    }

    fn greedy_rectangles(
        mut rows: [u32; CHUNK_WIDTH],
    ) -> impl Iterator<Item = (Vector2D<DiscreteUnit>, Vector2D<DiscreteUnit>)> {
        let mut r = 0;
        std::iter::from_fn(move || {
            while rows.get(r)? == &0 {
                r += 1;
            }
            let start = rows[r].trailing_zeros();
            let width = (rows[r] >> start).trailing_ones();
            let run = (((1u64 << width) - 1) << start) as u32;
            let height = rows[r..]
                .iter()
                .take_while(|&&row| row & run == run)
                .count();
            rows[r..r + height].iter_mut().for_each(|row| *row &= !run);
            Some((
                Vector2D {
                    x: start as DiscreteUnit,
                    y: r as DiscreteUnit,
                },
                Vector2D {
                    x: width as DiscreteUnit,
                    y: height as DiscreteUnit,
                },
            ))
        })
    }

    fn chunk_quads(&self, chunk: DiscretePoint) -> impl Iterator<Item = DiscreteQuad> {
        let solid = self.solid_columns(chunk);
        VoxelNormal::ALL.into_iter().flat_map(move |normal| {
            let axis = normal.axis();
            let (column, row) = axis.plane();
            let mut planes = [[0u32; CHUNK_WIDTH]; CHUNK_WIDTH];
            let mut occupied_layers = 0u32;
            for (r, columns) in solid[axis as usize].iter().enumerate() {
                for (c, &voxels) in columns.iter().enumerate() {
                    let exposed = if normal.is_positive() {
                        voxels & !(voxels >> 1)
                    } else {
                        voxels & !(voxels << 1)
                    };
                    let mut faces = (exposed >> 1) as u32 & CHUNK_MASK;
                    occupied_layers |= faces;
                    while faces != 0 {
                        planes[faces.trailing_zeros() as usize][r] |= 1 << c;
                        faces &= faces - 1;
                    }
                }
            }
            std::iter::from_fn(move || {
                (occupied_layers != 0).then(|| {
                    let layer = occupied_layers.trailing_zeros() as usize;
                    occupied_layers &= occupied_layers - 1;
                    layer
                })
            })
            .flat_map(move |layer| {
                Self::greedy_rectangles(planes[layer]).map(move |(start, size)| {
                    let mut origin = chunk;
                    origin[axis] += layer as DiscreteUnit;
                    origin[column] += start.x;
                    origin[row] += start.y;
                    VoxelQuad {
                        normal,
                        origin,
                        size,
                    }
                })
            })
        })
    }

    fn tesselate_chunk(&self, chunk: DiscretePoint) -> Vec<RealTriangle> {
        let voxel_size = self
            .config
            .as_ref()
            .expect("Tesselator must be configured before generating output geometry")
            .voxel_size as DiscreteUnit;
        if self.marching_cubes_enabled {
            return self.marching_cubes_chunk(chunk, voxel_size);
        }
        self.chunk_quads(chunk)
            .flat_map(|quad| quad.triangles())
            .map(|triangle| triangle.map(|coordinate| (coordinate * voxel_size) as RealUnit))
            .collect()
    }

    fn marching_cubes_chunk(
        &self,
        chunk: DiscretePoint,
        voxel_size: DiscreteUnit,
    ) -> Vec<RealTriangle> {
        let field = self.marching_cubes_field(chunk);
        let grid_index = |x: usize, y: usize, z: usize| (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;
        if !field.iter().any(|&value| value != 0) || field.iter().all(|&value| value != 0) {
            return Vec::new();
        }

        let mut triangles = Vec::new();
        for z in 0..CHUNK_WIDTH {
            for y in 0..CHUNK_WIDTH {
                for x in 0..CHUNK_WIDTH {
                    let corners = MC_CUBE_CORNERS.map(|offset| {
                        let cx = x + offset[0];
                        let cy = y + offset[1];
                        let cz = z + offset[2];
                        (
                            [
                                (chunk.x + cx as DiscreteUnit) as RealUnit,
                                (chunk.y + cy as DiscreteUnit) as RealUnit,
                                (chunk.z + cz as DiscreteUnit) as RealUnit,
                            ],
                            field[grid_index(cx, cy, cz)] as RealUnit,
                        )
                    });
                    let solid_corners = corners
                        .iter()
                        .filter(|corner| corner.1 > MC_ISO_LEVEL)
                        .count();
                    if solid_corners == 0 || solid_corners == MC_CUBE_CORNERS.len() {
                        continue;
                    }
                    for tetrahedron in MC_CUBE_TETRAHEDRA {
                        let points = tetrahedron.map(|corner| corners[corner].0);
                        let values = tetrahedron.map(|corner| corners[corner].1);
                        polygonize_tetrahedron(&points, values, voxel_size, &mut triangles);
                    }
                }
            }
        }
        triangles
    }

    fn marching_cubes_field(&self, chunk: DiscretePoint) -> Vec<u8> {
        let padded_index =
            |x: usize, y: usize, z: usize| (z * MC_PADDED_GRID_SIZE + y) * MC_PADDED_GRID_SIZE + x;
        let source_chunks: [Option<&Chunk>; 27] = std::array::from_fn(|index| {
            let x = (index % 3) as DiscreteUnit - 1;
            let y = ((index / 3) % 3) as DiscreteUnit - 1;
            let z = (index / 9) as DiscreteUnit - 1;
            self.chunks
                .get(&DiscretePoint {
                    x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                    y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                    z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                })
                .map(Box::as_ref)
        });
        let source_coordinate = |coordinate: usize| {
            if coordinate == 0 {
                (0, CHUNK_WIDTH - 1)
            } else if coordinate == MC_PADDED_GRID_SIZE - 1 {
                (2, 0)
            } else {
                (1, coordinate - 1)
            }
        };
        let mut voxels = vec![0u8; MC_PADDED_GRID_SIZE.pow(3)];
        for z in 0..MC_PADDED_GRID_SIZE {
            for y in 0..MC_PADDED_GRID_SIZE {
                for x in 0..MC_PADDED_GRID_SIZE {
                    let (source_x, local_x) = source_coordinate(x);
                    let (source_y, local_y) = source_coordinate(y);
                    let (source_z, local_z) = source_coordinate(z);
                    let source_index = source_x + source_y * 3 + source_z * 9;
                    voxels[padded_index(x, y, z)] =
                        source_chunks[source_index].is_some_and(|source| {
                            source.voxel(Point {
                                x: local_x,
                                y: local_y,
                                z: local_z,
                            })
                        }) as u8;
                }
            }
        }

        let grid_index = |x: usize, y: usize, z: usize| (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;
        let mut field = vec![0u8; MC_GRID_SIZE.pow(3)];
        // Corner density is a local occupancy average; any terrain voxel keeps all its corners inside.
        for z in 0..MC_GRID_SIZE {
            for y in 0..MC_GRID_SIZE {
                for x in 0..MC_GRID_SIZE {
                    let mut solid = 0;
                    for dz in 0..=1 {
                        for dy in 0..=1 {
                            for dx in 0..=1 {
                                solid += voxels[padded_index(x + dx, y + dy, z + dz)];
                            }
                        }
                    }
                    field[grid_index(x, y, z)] = solid;
                }
            }
        }
        field
    }
}

fn polygonize_tetrahedron(
    points: &[[RealUnit; 3]; 4],
    values: [RealUnit; 4],
    voxel_size: DiscreteUnit,
    triangles: &mut Vec<RealTriangle>,
) {
    let mut inside = [0usize; 4];
    let mut outside = [0usize; 4];
    let (mut inside_count, mut outside_count) = (0, 0);
    for (index, &value) in values.iter().enumerate() {
        if value > MC_ISO_LEVEL {
            inside[inside_count] = index;
            inside_count += 1;
        } else {
            outside[outside_count] = index;
            outside_count += 1;
        }
    }
    if inside_count == 0 || outside_count == 0 {
        return;
    }

    let center = |indices: &[usize; 4], count: usize| {
        let mut center = [0.0; 3];
        for &index in &indices[..count] {
            for axis in 0..3 {
                center[axis] += points[index][axis] / count as RealUnit;
            }
        }
        center
    };
    let inside_center = center(&inside, inside_count);
    let outside_center = center(&outside, outside_count);
    let interpolate = |a: usize, b: usize| {
        let t = ((MC_ISO_LEVEL - values[a]) / (values[b] - values[a])).clamp(0.0, 1.0);
        std::array::from_fn(|axis| points[a][axis] + (points[b][axis] - points[a][axis]) * t)
    };
    let mut add_triangle = |mut triangle: [[RealUnit; 3]; 3]| {
        let ab = [
            triangle[1][0] - triangle[0][0],
            triangle[1][1] - triangle[0][1],
            triangle[1][2] - triangle[0][2],
        ];
        let ac = [
            triangle[2][0] - triangle[0][0],
            triangle[2][1] - triangle[0][1],
            triangle[2][2] - triangle[0][2],
        ];
        let normal = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let direction = [
            outside_center[0] - inside_center[0],
            outside_center[1] - inside_center[1],
            outside_center[2] - inside_center[2],
        ];
        if normal[0] * direction[0] + normal[1] * direction[1] + normal[2] * direction[2] < 0.0 {
            triangle.swap(1, 2);
        }
        let scale = voxel_size as RealUnit;
        triangles.push(RealTriangle {
            v0: triangle[0].map(|coordinate| coordinate * scale),
            v1: triangle[1].map(|coordinate| coordinate * scale),
            v2: triangle[2].map(|coordinate| coordinate * scale),
        });
    };

    match (inside_count, outside_count) {
        (1, 3) => {
            let a = inside[0];
            add_triangle([
                interpolate(a, outside[0]),
                interpolate(a, outside[1]),
                interpolate(a, outside[2]),
            ]);
        }
        (3, 1) => {
            let a = outside[0];
            add_triangle([
                interpolate(a, inside[0]),
                interpolate(a, inside[1]),
                interpolate(a, inside[2]),
            ]);
        }
        (2, 2) => {
            let ac = interpolate(inside[0], outside[0]);
            let ad = interpolate(inside[0], outside[1]);
            let bc = interpolate(inside[1], outside[0]);
            let bd = interpolate(inside[1], outside[1]);
            add_triangle([ac, ad, bd]);
            add_triangle([ac, bd, bc]);
        }
        _ => unreachable!(),
    }
}

impl Tesselator for GreedyMeshTesselator {
    type Configuration = GreedyMeshTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(&mut self, geometry_delta: &Self::InputGeometryDelta) {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before updating input geometry"
        );
        for subchunk in geometry_delta {
            let origin = DiscretePoint {
                x: subchunk.x,
                y: subchunk.y,
                z: subchunk.z,
            };
            debug_assert!(
                Axis::ALL
                    .iter()
                    .all(|&axis| origin[axis].rem_euclid(SUBCHUNK_SIZE[axis] as DiscreteUnit) == 0),
                "Subchunk origin {:?} is not aligned to {:?}",
                origin,
                SUBCHUNK_SIZE,
            );
            let chunk = Chunk::origin_of(origin);
            let local =
                Point::from_fn(|axis| (origin[axis] - chunk[axis]) as usize / SUBCHUNK_SIZE[axis]);
            let previous = self
                .chunks
                .get(&chunk)
                .map_or(0, |stored| stored.subchunk_mask(local));
            if previous == subchunk.i {
                continue;
            }
            let stored = self.chunks.entry(chunk).or_insert_with(Chunk::new);
            stored.set_subchunk_mask(local, subchunk.i);
            if stored.solid_subchunks == 0 {
                self.chunks.remove(&chunk);
            }
            self.mark_subchunk_dirty(chunk, local, previous ^ subchunk.i);
        }
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before generating output geometry"
        );
        if self.dirty_queue.is_empty() {
            return None;
        }
        self.in_queue.clear();
        let dirty: Vec<DiscretePoint> = self.dirty_queue.drain(..).collect();
        #[cfg(feature = "multi-threaded")]
        let dirty = dirty.into_par_iter();
        #[cfg(not(feature = "multi-threaded"))]
        let dirty = dirty.into_iter();
        let output_delta = dirty
            .map(|chunk| TriangleChunk {
                index: chunk,
                triangles: self.tesselate_chunk(chunk),
            })
            .collect();
        Some(output_delta)
    }

    fn configure(&mut self, configuration: Self::Configuration) {
        self.config = Some(configuration);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mesher_with_voxel() -> GreedyMeshTesselator {
        let mut tesselator = GreedyMeshTesselator::new();
        tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size: 1 });
        let input = vec![PointI {
            x: 0,
            y: 0,
            z: 0,
            i: 1 << PointI::voxel_bit(Point { x: 1, y: 1, z: 0 }),
        }];
        tesselator.update_input_geometry(&input);
        tesselator
    }

    #[test]
    fn marching_cubes_corner_field_contains_each_terrain_voxel() {
        let tesselator = mesher_with_voxel();
        let field = tesselator.marching_cubes_field(DiscretePoint { x: 0, y: 0, z: 0 });
        let index = |x: usize, y: usize, z: usize| (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;

        for z in 0..=1 {
            for y in 1..=2 {
                for x in 1..=2 {
                    assert_ne!(field[index(x, y, z)], 0);
                }
            }
        }
    }

    #[test]
    fn marching_cubes_can_be_toggled_and_rebuilds_chunks() {
        let mut tesselator = mesher_with_voxel();
        let greedy = tesselator.get_output_geometry_delta().unwrap();
        let greedy_triangle_count: usize = greedy.iter().map(|chunk| chunk.triangles.len()).sum();

        tesselator.set_marching_cubes_enabled(true);
        assert!(tesselator.marching_cubes_enabled());
        let smoothed = tesselator.get_output_geometry_delta().unwrap();
        let smoothed_triangle_count: usize =
            smoothed.iter().map(|chunk| chunk.triangles.len()).sum();
        assert!(smoothed_triangle_count > greedy_triangle_count);
        assert!(tesselator.get_output_geometry_delta().is_none());

        tesselator.set_marching_cubes_enabled(false);
        assert!(!tesselator.marching_cubes_enabled());
        assert!(tesselator.get_output_geometry_delta().is_some());
    }

    #[test]
    fn marching_cubes_interpolates_from_corner_density() {
        let points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let mut triangles = Vec::new();
        polygonize_tetrahedron(&points, [2.0, 0.0, 0.0, 0.0], 1, &mut triangles);

        assert_eq!(triangles.len(), 1);
        assert!(
            triangles[0]
                .v0
                .iter()
                .chain(&triangles[0].v1)
                .chain(&triangles[0].v2)
                .any(|&coordinate| (coordinate - 0.75).abs() < f32::EPSILON)
        );
    }

    #[test]
    fn marching_cubes_dirties_diagonal_chunks_for_boundary_changes() {
        let mut tesselator = mesher_with_voxel();
        tesselator.set_marching_cubes_enabled(true);
        tesselator.get_output_geometry_delta().unwrap();

        let input = vec![PointI {
            x: 0,
            y: 0,
            z: 0,
            i: (1 << PointI::voxel_bit(Point { x: 1, y: 1, z: 0 }))
                | (1 << PointI::voxel_bit(Point { x: 0, y: 0, z: 0 })),
        }];
        tesselator.update_input_geometry(&input);
        let delta = tesselator.get_output_geometry_delta().unwrap();
        assert!(delta.iter().any(|chunk| chunk.index
            == DiscretePoint {
                x: -16,
                y: -16,
                z: 0
            }));
    }
}
