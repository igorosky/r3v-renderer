/**
 * Minimal test harness for the JS bindings.
 *
 * Every test case runs in its own Node.js process (see tests/js_bindings.rs),
 * so a failed check throws and ends the process. Only Node.js built-ins are
 * used, except for WebGPU, which Node.js lacks (see webgpu.mjs).
 *
 * usage: renderer_tests.mjs list | run <test>
 */

import "./webgpu.mjs";

import { readFileSync } from "node:fs";
import { join } from "node:path";
import process from "node:process";
import { fileURLToPath, pathToFileURL } from "node:url";

const MODULE = "r3v_renderer";

/** Directory of the wasm-pack output (`--target web`), as passed in by
 * tests/js_bindings.rs. When run by hand without it, pkg/ is used, where the
 * build in the header of examples/js/voxels_to_ppm.mjs puts it. */
const PKG_DIR =
    process.env.R3V_RENDERER_PKG ?? fileURLToPath(new URL("../../pkg", import.meta.url));

export const rr = await import(pathToFileURL(join(PKG_DIR, `${MODULE}.js`)).href);
rr.initSync({ module: readFileSync(join(PKG_DIR, `${MODULE}_bg.wasm`)) });

// ---------------------------------------------------------------- checks

export function fail(message) {
    throw new Error(message);
}

export function check(cond, message = "check failed") {
    if (!cond) {
        fail(message);
    }
}

export function fmt(value) {
    return JSON.stringify(value);
}

export function same(a, b) {
    return fmt(a) === fmt(b);
}

/** Fails unless fn throws an instance of exactly cls, or returns a Promise
 * rejecting with one, with `contains` in its message when given. A wasm trap
 * (a Rust panic, after which the module is in an undefined state) does not
 * count. Resolves to the error. */
export async function throws(fn, cls, contains) {
    try {
        await fn();
    } catch (e) {
        check(!(e instanceof WebAssembly.RuntimeError),
            `trapped (Rust panic) instead of throwing ${cls.name}: ${e.message}`);
        check(e?.constructor === cls, `threw ${e?.constructor?.name} (${e?.message ?? e}), not ${cls.name}`);
        check(contains === undefined || e.message.includes(contains),
            `${cls.name} "${e.message}" does not contain "${contains}"`);
        return e;
    }
    fail(`expected ${cls.name}, nothing was thrown`);
}

/** Checks that got is within eps of expected, element wise for arrays. */
export function close(got, expected, eps = 1e-4) {
    if (Array.isArray(expected)) {
        check(got?.length === expected.length, `${fmt(got)} is not like ${fmt(expected)}`);
        expected.forEach((e, k) => close(got[k], e, eps));
    } else {
        check(Math.abs(got - expected) <= eps, `${got} is not ${expected}`);
    }
}

// --------------------------------------------------------------- helpers

export async function makeRenderer(width, height) {
    const renderer = await rr.WgpuRenderer.create();
    renderer.configure(new rr.WgpuRendererConfiguration(width, height));
    return renderer;
}

export function makeTesselator(voxelSize) {
    const tesselator = new rr.CpuTesselator();
    tesselator.configure(new rr.CpuTesselatorConfiguration(voxelSize));
    return tesselator;
}

/** A tesselator holding one full 4x4x2 block at voxel (x, y, z), with the
 * delta it produced passed to renderer. */
export function addBlock(renderer, voxelSize, x, y, z) {
    const tesselator = makeTesselator(voxelSize);
    tesselator.update_input_geometry([new rr.PointI(x, y, z, 2 ** 32 - 1)]);
    renderer.update_geometry(tesselator.get_output_geometry_delta());
    return tesselator;
}

/** Renders twice, so the returned frame shows the scene as it is now: frames
 * are read back one call late. */
export async function renderNow(renderer, camera, width, height) {
    const state = camera.get_state(width / height);
    const frame = new Uint8Array(width * height * 4);
    await renderer.render(state, frame);
    await renderer.render(state, frame);
    return frame;
}

/** Whether the pixel at (x, y) is not black (alpha ignored). */
export function lit(frame, width, x, y) {
    const offset = (y * width + x) * 4;
    return frame[offset] !== 0 || frame[offset + 1] !== 0 || frame[offset + 2] !== 0;
}

/** Whether the pixel at (x, y) is bgr, opaque, within 1 per channel. */
export function pixelIs(frame, width, x, y, bgr) {
    const offset = (y * width + x) * 4;
    return [...bgr, 255].every((expected, k) => Math.abs(frame[offset + k] - expected) <= 1);
}

// ------------------------------------------------------------------ main

/** testCases: test functions, run by name. Resolves to the exit code. */
export async function main(testCases) {
    const byName = new Map(testCases.map((fn) => [fn.name, fn]));
    const args = process.argv.slice(2);
    if (args.length === 1 && args[0] === "list") {
        process.stdout.write([...byName.keys()].map((name) => `${name}\n`).join(""));
        return 0;
    }
    if (args.length === 2 && args[0] === "run") {
        const fn = byName.get(args[1]);
        if (fn === undefined) {
            console.error(`unknown test ${args[1]}`);
            return 2;
        }
        await fn();
        return 0;
    }
    console.error(`usage: ${process.argv[1]} list | run <test>`);
    return 2;
}
