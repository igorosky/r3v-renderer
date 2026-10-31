//! `stl::StlMesh` and `stl::read_bytes`.

use js_sys::{Array, ArrayBuffer, Uint8Array};
use wasm_bindgen::prelude::*;

use crate::{
    geometry::RealTriangle,
    shapes,
    stl::{self, StlMesh},
    urdf::{UrdfGeometry, Vector3},
};

use super::{
    JsTriangle, Result, array, error, f32_property, property, range_error, string, triangle,
    type_error, vector_property,
};

/// `triangles` as `StlMesh` reads them: an array of `TriangleLike`s.
fn triangles(value: &JsValue) -> Result<Vec<RealTriangle>> {
    array(value, &|| "triangles".to_owned())?
        .iter()
        .enumerate()
        .map(|(k, item)| triangle(&item, &|| format!("triangles[{k}]")))
        .collect()
}

/// A primitive geometry as `UrdfDescription` describes it.
fn primitive(value: &JsValue) -> Result<UrdfGeometry> {
    let what = || "geometry".to_owned();
    let kind = string(&property(value, "type", &what)?, &|| "geometry.type".to_owned())?;
    let positive = |key: &str| {
        let number = f32_property(value, key, &what)?;
        if number.is_finite() && number > 0.0 {
            Ok(number)
        } else {
            Err(range_error(format!(
                "geometry.{key} must be positive, got {number}"
            )))
        }
    };
    Ok(match kind.as_str() {
        "box" => {
            let [x, y, z] = vector_property(value, "size", &what)?;
            if ![x, y, z].iter().all(|edge| edge.is_finite() && *edge > 0.0) {
                return Err(range_error(format!(
                    "geometry.size must be positive, got [{x}, {y}, {z}]"
                )));
            }
            UrdfGeometry::Box {
                size: Vector3 { x, y, z },
            }
        }
        "sphere" => UrdfGeometry::Sphere {
            radius: positive("radius")?,
        },
        "cylinder" => UrdfGeometry::Cylinder {
            radius: positive("radius")?,
            length: positive("length")?,
        },
        "capsule" => UrdfGeometry::Capsule {
            radius: positive("radius")?,
            length: positive("length")?,
        },
        "mesh" => {
            return Err(range_error(
                "a mesh geometry is read from its file, with read_bytes",
            ));
        }
        _ => return Err(range_error(format!("unknown geometry type {kind:?}"))),
    })
}

/// Triangle mesh for `WgpuRenderer.upload_asset` (`stl::StlMesh`).
#[wasm_bindgen(js_name = StlMesh)]
pub struct JsStlMesh(pub(super) StlMesh);

#[wasm_bindgen(js_class = StlMesh)]
impl JsStlMesh {
    /// A mesh of `triangles`.
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "TriangleLike[]")] triangles: JsValue,
    ) -> Result<JsStlMesh> {
        Ok(Self(StlMesh {
            triangles: self::triangles(&triangles)?,
        }))
    }

    /// `stl::read_bytes`: reads the contents of an ASCII or binary STL file
    /// and winds every triangle to match the normal declared for it. `Error`
    /// when they are no STL.
    pub fn read_bytes(
        #[wasm_bindgen(unchecked_param_type = "Uint8Array | ArrayBuffer")] bytes: JsValue,
    ) -> Result<JsStlMesh> {
        let bytes = if let Some(bytes) = bytes.dyn_ref::<Uint8Array>() {
            bytes.to_vec()
        } else if let Some(buffer) = bytes.dyn_ref::<ArrayBuffer>() {
            Uint8Array::new(buffer).to_vec()
        } else {
            return Err(type_error("bytes must be a Uint8Array or an ArrayBuffer"));
        };
        stl::read_bytes(&bytes).map(Self).map_err(error)
    }

    /// `shapes::tessellate`: a mesh of a box, sphere, cylinder or capsule
    /// centered on its origin, the cylinder and capsule along Z, as URDF
    /// places them. `geometry` is a `UrdfGeometry`, e.g. of a visual of
    /// `UrdfDescription.links`. `RangeError` for a mesh geometry, which is
    /// read from its file (`read_bytes`), and for sizes that are not
    /// positive.
    pub fn from_geometry(
        #[wasm_bindgen(unchecked_param_type = "UrdfGeometry")] geometry: JsValue,
    ) -> Result<JsStlMesh> {
        let geometry = primitive(&geometry)?;
        Ok(Self(
            shapes::tessellate(&geometry).expect("a primitive has triangles"),
        ))
    }

    /// The triangles, as copies: assign an array to change them.
    #[wasm_bindgen(getter, unchecked_return_type = "Triangle[]")]
    pub fn triangles(&self) -> Array {
        self.0
            .triangles
            .iter()
            .map(|&triangle| JsValue::from(JsTriangle(triangle)))
            .collect()
    }

    #[wasm_bindgen(setter)]
    pub fn set_triangles(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "TriangleLike[]")] triangles: JsValue,
    ) -> Result<()> {
        self.0.triangles = self::triangles(&triangles)?;
        Ok(())
    }
}
