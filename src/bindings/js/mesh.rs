//! `mesh::Mesh` and `stl::read_bytes`.

use js_sys::{Array, ArrayBuffer, Uint8Array};
use wasm_bindgen::prelude::*;

use crate::{geometry::RealTriangle, mesh::Mesh, stl};

use super::{JsTriangle, Result, array, error, triangle, type_error};

/// `triangles` as `Mesh` reads them: an array of `TriangleLike`s.
fn triangles(value: &JsValue) -> Result<Vec<RealTriangle>> {
    array(value, &|| "triangles".to_owned())?
        .iter()
        .enumerate()
        .map(|(k, item)| triangle(&item, &|| format!("triangles[{k}]")))
        .collect()
}

/// Triangle mesh for `WgpuRenderer.upload_asset` (`mesh::Mesh`).
#[wasm_bindgen(js_name = Mesh)]
pub struct JsMesh(pub(super) Mesh);

#[wasm_bindgen(js_class = Mesh)]
impl JsMesh {
    /// A mesh of `triangles`.
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "TriangleLike[]")] triangles: JsValue,
    ) -> Result<JsMesh> {
        Ok(Self(Mesh {
            triangles: self::triangles(&triangles)?,
            uvs: None,
        }))
    }

    /// `stl::read_bytes`: reads the contents of an ASCII or binary STL file
    /// and winds every triangle to match the normal declared for it. `Error`
    /// when they are no STL.
    pub fn read_stl_bytes(
        #[wasm_bindgen(unchecked_param_type = "Uint8Array | ArrayBuffer")] bytes: JsValue,
    ) -> Result<JsMesh> {
        let bytes = if let Some(bytes) = bytes.dyn_ref::<Uint8Array>() {
            bytes.to_vec()
        } else if let Some(buffer) = bytes.dyn_ref::<ArrayBuffer>() {
            Uint8Array::new(buffer).to_vec()
        } else {
            return Err(type_error("bytes must be a Uint8Array or an ArrayBuffer"));
        };
        stl::read_bytes(&bytes).map(Self).map_err(error)
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
        self.0.uvs = None;
        Ok(())
    }
}
