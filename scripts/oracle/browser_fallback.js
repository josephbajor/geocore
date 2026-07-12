// Cookie-session fallback for the Onshape oracle loop (docs/oracle-loop.md).
//
// The API-key CLI (scripts/oracle_loop.py) is the primary path. Use this only
// when no key pair is provisioned: paste it into the devtools console (or
// inject it with browser tooling) on a LOGGED-IN Onshape document page, then:
//
//   await __oracle.run("solid_block.x_t", "<base64 of the .x_t bytes>")
//   -> { name, bytes, state, reason }
//
// Document/workspace/element ids are read from the page URL; override with
// __oracle.configure({ did, wid, eid }) when injecting from another page.
// Session-expiry tell: GET /api/users/sessioninfo returns 204 (anonymous)
// even though the public page still renders; a human must sign in again.
(() => {
  // The XSRF token may end in base64 '=' padding: slice off the cookie name,
  // never split('=')[1], or POSTs fail 401 with an empty body.
  const cookie = document.cookie.split("; ").find((c) => c.startsWith("XSRF-TOKEN="));
  const xsrf = cookie ? cookie.slice("XSRF-TOKEN=".length) : null;
  const ids = location.pathname.match(
    /documents\/([0-9a-f]{24})\/w\/([0-9a-f]{24})\/e\/([0-9a-f]{24})/,
  );

  window.__oracle = {
    did: ids?.[1],
    wid: ids?.[2],
    eid: ids?.[3],
    xsrf,

    configure({ did, wid, eid }) {
      Object.assign(this, { did, wid, eid });
      return this;
    },

    // Upload one .x_t (base64) and poll its auto-started translation to a
    // terminal state. DONE = accepted; FAILED carries the host failureReason
    // whose wording localizes the defect (see docs/oracle-loop.md taxonomy).
    async run(name, b64, { attempts = 150, delayMs = 2000 } = {}) {
      if (!this.did || !this.wid || !this.eid) {
        return { name, bytes: 0, state: "CONFIG", reason: "call configure({did, wid, eid})" };
      }
      if (!this.xsrf) {
        return { name, bytes: 0, state: "AUTH", reason: "no XSRF-TOKEN cookie; sign in first" };
      }
      const bytes = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
      const form = new FormData();
      form.append("file", new Blob([bytes]), name);
      const post = await fetch(
        `/api/v6/blobelements/d/${this.did}/w/${this.wid}/e/${this.eid}`,
        {
          method: "POST",
          headers: { "X-XSRF-TOKEN": this.xsrf },
          body: form,
          credentials: "same-origin",
        },
      );
      if (!post.ok) {
        return { name, bytes: bytes.length, state: `HTTP ${post.status}`, reason: await post.text() };
      }
      const { translationId } = await post.json();
      for (let i = 0; i < attempts; i += 1) {
        const poll = await fetch(`/api/v6/translations/${translationId}`, {
          credentials: "same-origin",
        });
        const status = await poll.json();
        if (status.requestState !== "ACTIVE") {
          return {
            name,
            bytes: bytes.length,
            state: status.requestState,
            reason: status.failureReason ?? "",
          };
        }
        await new Promise((resolve) => setTimeout(resolve, delayMs));
      }
      return { name, bytes: bytes.length, state: "TIMEOUT", reason: "" };
    },
  };
})();
