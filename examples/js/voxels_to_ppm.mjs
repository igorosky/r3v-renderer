/**
 * Renders a staircase of voxels and a triangle mesh with the JS bindings and
 * writes the frame to a binary PPM image.
 *
 *     wasm-pack build --release --target web --features js
 *     (cd tests/js && npm ci)
 *     node --import ./tests/js/webgpu.mjs examples/js/voxels_to_ppm.mjs stairs.ppm
 *
 * Node.js has no WebGPU, tests/js/webgpu.mjs provides it. The module is loaded
 * from pkg/, the output of wasm-pack, or from R3V_RENDERER_PKG when set.
 */

import { readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import process from "node:process";
import { fileURLToPath, pathToFileURL } from "node:url";

const PKG_DIR =
    process.env.R3V_RENDERER_PKG ?? fileURLToPath(new URL("../../pkg", import.meta.url));
const rr = await import(pathToFileURL(join(PKG_DIR, "r3v_renderer.js")).href);
rr.initSync({ module: readFileSync(join(PKG_DIR, "r3v_renderer_bg.wasm")) });

const [WIDTH, HEIGHT] = [320, 240];
const STEPS = 8;

/** Feeds a staircase of full 4x4x2 blocks through the tesselator into the
 * renderer: step i is i / 2 + 1 blocks high (Y is up) and 4 blocks deep. The
 * blocks could just as well come from r3v-voxelizer in compress mode. */
function addStairs(renderer) {
    const tesselator = new rr.CpuTesselator();
    tesselator.configure(new rr.CpuTesselatorConfiguration(0.25));
    const blocks = [];
    for (let step = 0; step < STEPS; step++) {
        for (let level = 0; level <= Math.floor(step / 2); level++) {
            for (let depth = 0; depth < 4; depth++) {
                blocks.push(new rr.PointI(4 * step, 4 * level, 2 * depth, 2 ** 32 - 1));
            }
        }
    }
    tesselator.update_input_geometry(blocks);
    const delta = tesselator.get_output_geometry_delta();
    if (delta !== undefined) {
        renderer.update_geometry(delta);
    }
}

/** Two instances of one triangle above the stairs. */
function addFlags(renderer) {
    const mesh = new rr.StlMesh([new rr.Triangle([0, 0, 0], [1, 0, 0], [0, 1, 0])]);
    renderer.upload_asset(mesh, 2).forEach((instance, k) => {
        renderer.set_asset_transform(instance, new rr.Twist([1 + 4 * k, 3, 2.5]));
    });
}

function writePpm(path, rgba) {
    const rgb = new Uint8Array(WIDTH * HEIGHT * 3);
    for (let p = 0; p < WIDTH * HEIGHT; p++) {
        rgb.set(rgba.subarray(4 * p, 4 * p + 3), 3 * p);
    }
    writeFileSync(path, Buffer.concat([Buffer.from(`P6\n${WIDTH} ${HEIGHT}\n255\n`), rgb]));
}

async function main(args) {
    if (args.length !== 1) {
        console.error(`usage: ${process.argv[1]} <output.ppm>`);
        return 2;
    }

    const renderer = await rr.WgpuRenderer.create();
    renderer.configure(new rr.WgpuRendererConfiguration(WIDTH, HEIGHT));
    addStairs(renderer);
    addFlags(renderer);

    const camera = new rr.OrbitCamera([4, 1, 1], 12, 30, 25);
    const state = camera.get_state(WIDTH / HEIGHT);

    // Frames come back one call late, the first call gives zeros. RGBA, as
    // ImageData would take it in a browser.
    const frame = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
    await renderer.render_rgba(state, frame);
    await renderer.render_rgba(state, frame);

    writePpm(args[0], frame);
    return 0;
}

process.exit(await main(process.argv.slice(2)));
