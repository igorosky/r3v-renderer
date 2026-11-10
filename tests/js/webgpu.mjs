/**
 * Gives Node.js the WebGPU that browsers have, from the `webgpu` package
 * (Dawn), installed into tests/js/node_modules by `npm ci` in tests/js.
 *
 * Imported by the test harness, and preloaded for the example:
 *
 *     node --import ./tests/js/webgpu.mjs examples/js/voxels_to_ppm.mjs out.ppm
 */

import { create, globals } from "webgpu";

/** Every device requested through navigator.gpu, for gpuIdle. */
const devices = [];

const gpu = create([]);
// Records the devices wgpu requests. Own properties shadow the native
// methods, which are called on the native objects as before.
const requestAdapter = gpu.requestAdapter;
Object.defineProperty(gpu, "requestAdapter", {
    value: async (...args) => {
        const adapter = await requestAdapter.apply(gpu, args);
        if (adapter !== null) {
            const requestDevice = adapter.requestDevice;
            Object.defineProperty(adapter, "requestDevice", {
                value: async (...args) => {
                    const device = await requestDevice.apply(adapter, args);
                    devices.push(device);
                    return device;
                },
            });
        }
        return adapter;
    },
});

Object.assign(globalThis, globals);
Object.defineProperty(globalThis.navigator, "gpu", { value: gpu, configurable: true });
// wgpu only looks for navigator.gpu when the global object is a Window or a
// WorkerGlobalScope, which it finds by those constructors.
globalThis.Window ??= class Window {};

/**
 * Resolves once every device has finished the work submitted to it.
 *
 * A render resolves before the GPU finished the frame it submitted (frames
 * are read back one call late), and a software Vulkan driver such as
 * lavapipe may still be compiling that frame's shaders then. Ending the
 * process meanwhile runs the exit handlers, which free LLVM's global state,
 * under the compiling thread: it crashes (SIGSEGV, or a heap corruption
 * abort) now and then. Await this before process.exit().
 */
export async function gpuIdle() {
    await Promise.allSettled(devices.map((device) => device.queue.onSubmittedWorkDone()));
}
