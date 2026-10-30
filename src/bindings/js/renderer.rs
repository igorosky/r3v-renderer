//! `renderer::WgpuAccelerated` (trait `Renderer`).

use std::{cell::Cell, rc::Rc};

use js_sys::{Array, Promise, Uint8Array, Uint8ClampedArray};
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::future_to_promise;

use crate::{
    bindings::checked::CheckedRenderer,
    camera::CameraState,
    renderer::{DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE},
};

use super::{
    Result, U32_MAX, error, f32_property, index, integer_property, number, numbers, property,
    range_error, stl::JsStlMesh, tesselator::JsGeometryDelta, twist, type_error, vector,
    vector_property,

};

/// `renderer::WgpuAcceleratedConfiguration`: the frame size in pixels, the
/// opaque sRGB background color (components between 0 and 1) and the
/// distance along the camera's forward axis over which the voxel colormap
/// runs once.
#[wasm_bindgen(js_name = WgpuRendererConfiguration)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsWgpuRendererConfiguration {
    pub width: u32,
    pub height: u32,
    background_color: [f32; 3],
    pub color_depth_range: f32,
}

#[wasm_bindgen(js_class = WgpuRendererConfiguration)]
impl JsWgpuRendererConfiguration {
    /// Without them the background is black and the depth range 50.
    #[wasm_bindgen(constructor)]
    pub fn new(
        width: u32,
        height: u32,
        #[wasm_bindgen(unchecked_optional_param_type = "Vec3")] background_color: Option<JsValue>,
        #[wasm_bindgen(unchecked_optional_param_type = "number")] color_depth_range: Option<
            JsValue,
        >,
    ) -> Result<JsWgpuRendererConfiguration> {
        Ok(Self {
            width,
            height,
            background_color: match background_color {
                Some(color) => vector(&color, &|| "background_color".to_owned())?,
                None => DEFAULT_BACKGROUND_COLOR,
            },
            color_depth_range: match color_depth_range {
                Some(range) => number(&range, &|| "color_depth_range".to_owned())? as f32,
                None => DEFAULT_COLOR_DEPTH_RANGE,
            },
        })
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn background_color(&self) -> Array {
        numbers(&self.background_color)
    }

    #[wasm_bindgen(setter)]
    pub fn set_background_color(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] background_color: JsValue,
    ) -> Result<()> {
        self.background_color = vector(&background_color, &|| "background_color".to_owned())?;
        Ok(())
    }
}

struct Rendering {
    renderer: CheckedRenderer,
    // Rendered into, then copied to the caller's array
    frame: Vec<u8>,
}

/// `out` of `render` as bytes: a `Uint8Array`, or a view on the memory of a
/// `Uint8ClampedArray`.
fn bytes_of(out: &JsValue, frame_len: usize) -> Result<Uint8Array> {
    let (bytes, len) = if let Some(bytes) = out.dyn_ref::<Uint8Array>() {
        (Some(bytes), bytes.length())
    } else if let Some(clamped) = out.dyn_ref::<Uint8ClampedArray>() {
        (None, clamped.length())
    } else {
        return Err(type_error(
            "`out` must be a Uint8Array or a Uint8ClampedArray",
        ));
    };
    // Also before a view is made: a detached array has no memory to view
    if (len as usize) < frame_len {
        return Err(range_error(format!(
            "a frame needs {frame_len} bytes, `out` has {len}"
        )));
    }
    Ok(match bytes {
        Some(bytes) => bytes.clone(),
        None => {
            let clamped = out.unchecked_ref::<Uint8ClampedArray>();
            Uint8Array::new_with_byte_offset_and_length(
                &clamped.buffer(),
                clamped.byte_offset(),
                len,
            )
        }
    })
}

impl Rendering {
    /// Renders into `out`, which is checked to be large enough.
    async fn render(
        &mut self,
        camera_state: &CameraState,
        out: &Uint8Array,
        rgba: bool,
    ) -> Result<()> {
        let frame_len = self.renderer.frame_len()?;
        self.frame.resize(frame_len, 0);
        self.renderer
            .render_async(camera_state, &mut self.frame)
            .await?;
        if rgba {
            let (pixels, _) = self.frame.as_chunks_mut::<4>();
            for pixel in pixels {
                pixel.swap(0, 2);
            }
        }
        // JS ran meanwhile, it may have detached (e.g. transferred) the array
        if (out.length() as usize) < frame_len {
            return Err(error("`out` was detached or shrunk while rendering"));
        }
        out.subarray(0, frame_len as u32).copy_from(&self.frame);
        Ok(())
    }
}

/// Renders voxel geometry and mesh instances on WebGPU
/// (`renderer::WgpuAccelerated`). Created with the async `create`, configure
/// it before rendering.
///
/// `render` returns a Promise. Until it settles the renderer is busy: every
/// other call throws, or rejects for `render`.
#[wasm_bindgen(js_name = WgpuRenderer)]
pub struct JsWgpuRenderer(Rc<Cell<Option<Rendering>>>);

impl JsWgpuRenderer {
    /// The renderer, taken out until it is put back. `render` keeps it until
    /// its Promise settles, so that calls made meanwhile find none.
    fn take(&self) -> Result<Rendering> {
        self.0.take().ok_or_else(|| {
            error("WgpuRenderer.render is still running, await it before using the renderer")
        })
    }

    fn with<T>(&self, f: impl FnOnce(&mut CheckedRenderer) -> Result<T>) -> Result<T> {
        let mut rendering = self.take()?;
        let result = f(&mut rendering.renderer);
        self.0.set(Some(rendering));
        result
    }

    /// Checks the arguments and takes the renderer before it returns, renders
    /// in the Promise. Errors are rejections either way.
    fn render_frame(&self, camera_state: &JsValue, out: &JsValue, rgba: bool) -> Promise {
        let start = || -> Result<_> {
            let camera_state = super::camera_state(camera_state, &|| "camera_state".to_owned())?;
            let out = self.with(|renderer| bytes_of(out, renderer.frame_len()?))?;
            Ok((camera_state, out, self.take()?))
        };
        let (camera_state, out, mut rendering) = match start() {
            Ok(started) => started,
            Err(error) => return Promise::reject(&error),
        };
        let slot = self.0.clone();
        future_to_promise(async move {
            let result = rendering.render(&camera_state, &out, rgba).await;
            slot.set(Some(rendering));
            result.map(|()| JsValue::UNDEFINED)
        })
    }
}

#[wasm_bindgen(js_class = WgpuRenderer)]
impl JsWgpuRenderer {
    /// `WgpuAccelerated::new_async`: a renderer on a device of the default
    /// WebGPU adapter. Rejects when there is no adapter or no device.
    pub async fn create() -> Result<JsWgpuRenderer> {
        let renderer = CheckedRenderer::new_async().await?;
        Ok(Self(Rc::new(Cell::new(Some(Rendering {
            renderer,
            frame: Vec::new(),
        })))))
    }

    /// `Renderer::configure`: sets the frame size, the background color and
    /// the colormap depth range (black and 50 when left out). `RangeError`
    /// unless both sides are between 1 and 8192, the color components between
    /// 0 and 1 and the depth range positive.
    pub fn configure(
        &self,
        #[wasm_bindgen(
            unchecked_param_type = "{ width: number; height: number; background_color?: Vec3; color_depth_range?: number }"
        )]
        configuration: JsValue,
    ) -> Result<()> {
        let what = || "configuration".to_owned();
        let width = integer_property(&configuration, "width", 0.0, U32_MAX, &what)? as u32;
        let height = integer_property(&configuration, "height", 0.0, U32_MAX, &what)? as u32;
        let background_color =
            if property(&configuration, "background_color", &what)?.is_undefined() {
                DEFAULT_BACKGROUND_COLOR
            } else {
                vector_property(&configuration, "background_color", &what)?
            };
        let color_depth_range =
            if property(&configuration, "color_depth_range", &what)?.is_undefined() {
                DEFAULT_COLOR_DEPTH_RANGE
            } else {
                f32_property(&configuration, "color_depth_range", &what)?
            };
        self.with(|renderer| {
            Ok(renderer.configure(width, height, background_color, color_depth_range)?)
        })
    }

    /// `Renderer::update_geometry`: replaces the triangles of every chunk in
    /// `delta`, which can not be used afterwards.
    pub fn update_geometry(&self, delta: &mut JsGeometryDelta) -> Result<()> {
        self.with(|renderer| {
            renderer.update_geometry(delta.take()?);
            Ok(())
        })
    }

    /// `Renderer::upload_asset`: uploads a copy of `mesh` with `instances`
    /// instances and returns their ids. Instances start visible, with the
    /// identity transform. Ids count up from 0 over all uploads.
    #[wasm_bindgen(unchecked_return_type = "number[]")]
    pub fn upload_asset(
        &self,
        mesh: &JsStlMesh,
        #[wasm_bindgen(unchecked_param_type = "number")] instances: JsValue,
    ) -> Result<Array> {
        let instances = index(&instances, &|| "instances".to_owned())?;
        let ids = self.with(|renderer| Ok(renderer.upload_asset(mesh.0.clone(), instances)?))?;
        Ok(ids
            .into_iter()
            .map(|id| JsValue::from_f64(id as f64))
            .collect())
    }

    /// `Renderer::set_asset_transform`. `RangeError` for an id that was never
    /// returned by `upload_asset`.
    pub fn set_asset_transform(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number")] instance_id: JsValue,
        #[wasm_bindgen(unchecked_param_type = "TwistLike")] transform: JsValue,
    ) -> Result<()> {
        let instance_id = index(&instance_id, &|| "instance_id".to_owned())?;
        let transform = twist(&transform, &|| "transform".to_owned())?;
        self.with(|renderer| Ok(renderer.set_asset_transform(instance_id, transform)?))
    }

    /// `Renderer::set_asset_visibility`. `RangeError` for an id that was never
    /// returned by `upload_asset`.
    pub fn set_asset_visibility(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number")] instance_id: JsValue,
        visible: bool,
    ) -> Result<()> {
        let instance_id = index(&instance_id, &|| "instance_id".to_owned())?;
        self.with(|renderer| Ok(renderer.set_asset_visibility(instance_id, visible)?))
    }

    pub fn set_asset_color(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number")] instance_id: JsValue,
        #[wasm_bindgen(unchecked_param_type = "[number, number, number]")] color: JsValue,
    ) -> Result<()> {
        let instance_id = index(&instance_id, &|| "instance_id".to_owned())?;
        let color = vector(&color, &|| "color".to_owned())?;
        self.with(|renderer| Ok(renderer.set_asset_color(instance_id, color)?))
    }

    /// `WgpuAccelerated::render_async`: renders a frame seen by
    /// `camera_state` and writes the previous one to `out` (frames are read
    /// back one call late). After `configure` changed the frame size, and on
    /// the first call, `out` is filled with zeros instead.
    ///
    /// Pixels are BGRA, 8 bit sRGB, rows top to bottom without padding, as in
    /// the other bindings; `render_rgba` gives RGBA. `out` is a `Uint8Array`
    /// or a `Uint8ClampedArray` with room for `width * height * 4` bytes.
    #[wasm_bindgen(unchecked_return_type = "Promise<void>")]
    pub fn render(
        &self,
        #[wasm_bindgen(unchecked_param_type = "CameraStateLike")] camera_state: &JsValue,
        #[wasm_bindgen(unchecked_param_type = "Uint8Array | Uint8ClampedArray")] out: &JsValue,
    ) -> Promise {
        self.render_frame(camera_state, out, false)
    }

    /// `render` with RGBA pixels, as `ImageData` takes them: the channels are
    /// swapped before the frame is written to `out`.
    #[wasm_bindgen(unchecked_return_type = "Promise<void>")]
    pub fn render_rgba(
        &self,
        #[wasm_bindgen(unchecked_param_type = "CameraStateLike")] camera_state: &JsValue,
        #[wasm_bindgen(unchecked_param_type = "Uint8Array | Uint8ClampedArray")] out: &JsValue,
    ) -> Promise {
        self.render_frame(camera_state, out, true)
    }
}
