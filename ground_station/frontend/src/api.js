// Keep validation errors readable without echoing the rejected request body
// (which may contain credentials in FastAPI's `input` field).
export function formatApiError(payload, status) {
  const describe = (detail) => {
    if (typeof detail === "string") return detail;
    if (!detail || typeof detail !== "object") return "";
    const message = typeof detail.msg === "string" ? detail.msg
      : typeof detail.message === "string" ? detail.message : "";
    const location = Array.isArray(detail.loc)
      ? detail.loc.filter((part) => typeof part === "string" || typeof part === "number").join(".") : "";
    return message ? (location ? `${location}: ${message}` : message) : "";
  };
  const detail = payload?.detail;
  const message = Array.isArray(detail)
    ? detail.map(describe).filter(Boolean).join("；") : describe(detail);
  return `HTTP ${status}：${message || "请求失败，服务器未返回可显示的错误说明。"}`;
}

export async function api(path, options = {}, fetchImpl = globalThis.fetch) {
  const { headers: suppliedHeaders, ...requestOptions } = options;
  const headers = new Headers(suppliedHeaders);
  if (!headers.has("Content-Type")) headers.set("Content-Type", "application/json");
  // Set headers after options so an operator-token header cannot erase JSON type.
  const response = await fetchImpl(path, { ...requestOptions, headers });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(formatApiError(payload, response.status));
  return payload;
}
