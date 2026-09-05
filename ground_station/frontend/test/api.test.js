import test from "node:test";
import assert from "node:assert/strict";
import { api, formatApiError } from "../src/api.js";

// All fetches are mocked. These tests must never contact an actual control API.
for (const [name, suppliedHeaders] of [
  ["empty headers", {}],
  ["operator token", { "X-Operator-Token": "test-only-token" }],
  ["Headers instance", new Headers({ "X-Operator-Token": "test-only-token" })],
  ["header pairs", [["X-Operator-Token", "test-only-token"]]],
]) {
  test(`JSON content type survives ${name}`, async () => {
    const body = JSON.stringify({ mode: "dry_run", category: "atomic", atomic_task: "takeoff" });
    let calls = 0;
    const result = await api("/api/tasks/dispatch", { method: "POST", headers: suppliedHeaders, body }, async (path, init) => {
      calls++;
      assert.equal(path, "/api/tasks/dispatch");
      assert.equal(init.method, "POST");
      assert.equal(init.body, body);
      const request = new Request("https://offline.invalid", init); // Construct only; never fetch.
      assert.equal(request.headers.get("content-type"), "application/json");
      assert.equal(request.headers.get("x-operator-token"), name === "empty headers" ? null : "test-only-token");
      return { ok: true, json: async () => ({ status: "mock_success" }) };
    });
    assert.deepEqual(result, { status: "mock_success" });
    assert.equal(calls, 1);
  });
}

test("explicit content type remains intact", async () => {
  await api("/mock", { headers: { "content-type": "application/custom+json" } }, async (_, init) => {
    assert.equal(init.headers.get("content-type"), "application/custom+json");
    return { ok: true, json: async () => ({}) };
  });
});

test("FastAPI errors expose field and message, never input/credentials", () => {
  const message = formatApiError({ detail: [
    { loc: ["body", "parameters", "distance_m"], msg: "Input should be greater than or equal to 0.05", input: "SECRET" },
    { loc: ["body"], msg: "Input should be a valid dictionary", ctx: { token: "SECRET" } },
  ] }, 422);
  assert.match(message, /HTTP 422/);
  assert.match(message, /body.parameters.distance_m/);
  assert.match(message, /valid dictionary/);
  assert.ok(!message.includes("[object Object]"));
  assert.ok(!message.includes("SECRET"));
});

test("string and single-object errors are readable", () => {
  assert.equal(formatApiError({ detail: "Invalid operator control token." }, 401), "HTTP 401：Invalid operator control token.");
  assert.equal(formatApiError({ detail: { msg: "Invalid body", loc: ["body"] } }, 422), "HTTP 422：body: Invalid body");
});

test("unknown structured details have a safe fallback", () => {
  const message = formatApiError({ detail: { input: "SECRET" } }, 500);
  assert.match(message, /HTTP 500/);
  assert.ok(!message.includes("SECRET"));
  assert.ok(!message.includes("[object Object]"));
});

test("rejected request is displayed once and never retried", async () => {
  let calls = 0;
  await assert.rejects(api("/mock", {}, async () => {
    calls++;
    return { ok: false, status: 422, json: async () => ({ detail: [{ loc: ["body"], msg: "Invalid body" }] }) };
  }), /HTTP 422：body: Invalid body/);
  assert.equal(calls, 1);
});

test("non-JSON error body falls back to HTTP status", async () => {
  await assert.rejects(api("/mock", {}, async () => ({
    ok: false, status: 502, json: async () => { throw new SyntaxError("HTML body"); },
  })), /HTTP 502/);
});
