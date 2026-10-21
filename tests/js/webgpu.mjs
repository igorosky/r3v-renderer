/**
 * Gives Node.js the WebGPU that browsers have, from the `webgpu` package
 * (Dawn), installed into tests/js/node_modules by `npm ci` in tests/js.
 *
 * Imported by the test harness, and preloaded for the example:
 *
 *     node --import ./tests/js/webgpu.mjs examples/js/voxels_to_ppm.mjs out.ppm
 */

import { create, globals } from "webgpu";

Object.assign(globalThis, globals);
Object.defineProperty(globalThis.navigator, "gpu", { value: create([]), configurable: true });
// wgpu only looks for navigator.gpu when the global object is a Window or a
// WorkerGlobalScope, which it finds by those constructors.
globalThis.Window ??= class Window {};
