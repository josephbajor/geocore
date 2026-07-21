"""Stdlib-only Onshape REST client for the licensed-host oracle loop.

Authentication is an Onshape developer API key pair sent as HTTP Basic
credentials, so the loop runs identically for a human at a terminal, an
agent session, or a bounded CI job — no browser, cookies, or XSRF handling.
The blob-upload/translation-poll pair and the failure-reason taxonomy are
host-proven (docs/oracle-loop.md); the export-back pair follows Onshape's
published translation API and is confirmed against the live host on first
use.
"""

from __future__ import annotations

import base64
import json
import os
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_BASE_URL = "https://cad.onshape.com"
ACCESS_KEY_ENV = "ONSHAPE_ACCESS_KEY"
SECRET_KEY_ENV = "ONSHAPE_SECRET_KEY"
BASE_URL_ENV = "ONSHAPE_BASE_URL"
REQUEST_LIMIT_ENV = "ONSHAPE_REQUEST_LIMIT"
MAX_REQUEST_LIMIT = 400

CONFIG_PATH = Path("oracle/config.json")
CONFIG_KEYS = ("document_id", "workspace_id", "element_id")

TERMINAL_STATES = ("DONE", "FAILED")

# Host failure-reason taxonomy: the reason string localizes a writer defect
# to framing/layout vs topology/geometry semantics vs late translation
# before any bisection starts (docs/oracle-loop.md).
PARSE_LEVEL = "parse-level"
SEMANTIC = "semantic"
LATE_TRANSLATION = "late-translation"


class OracleError(Exception):
    """Authentication, transport, configuration, or protocol failure."""


def load_env_file(path):
    """Apply KEY=VALUE lines from an untracked .env without overriding the
    real environment. Returns the keys applied."""
    applied = []
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return applied
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, _, value = stripped.partition("=")
        key = key.strip()
        value = value.strip().strip("'\"")
        if key and key not in os.environ:
            os.environ[key] = value
            applied.append(key)
    return applied


def classify_failure(reason):
    """Map a host failureReason string onto the documented taxonomy."""
    if not reason:
        return ""
    lowered = reason.lower()
    if "corrupt" in lowered or "invalid" in lowered:
        return PARSE_LEVEL
    if "no translatable geometry" in lowered:
        return SEMANTIC
    return LATE_TRANSLATION


def encode_multipart(field_name, filename, payload, boundary=None):
    """Encode one file field as multipart/form-data.

    Returns a `(content_type, body)` pair. The boundary parameter exists for
    deterministic tests; live callers omit it.
    """
    boundary = boundary or uuid.uuid4().hex
    head = (
        "--{boundary}\r\n"
        'Content-Disposition: form-data; name="{name}"; filename="{file}"\r\n'
        "Content-Type: application/octet-stream\r\n"
        "\r\n"
    ).format(boundary=boundary, name=field_name, file=filename)
    tail = "\r\n--{boundary}--\r\n".format(boundary=boundary)
    body = head.encode("utf-8") + bytes(payload) + tail.encode("utf-8")
    return "multipart/form-data; boundary={}".format(boundary), body


class ApiKeyTransport:
    """Basic-auth HTTP transport bound to one Onshape deployment."""

    def __init__(
        self,
        access_key,
        secret_key,
        base_url=DEFAULT_BASE_URL,
        timeout=60.0,
        request_limit=None,
    ):
        if not access_key or not secret_key:
            raise OracleError(
                "missing Onshape API key pair: set {} and {} "
                "(create keys at https://dev-portal.onshape.com)".format(
                    ACCESS_KEY_ENV, SECRET_KEY_ENV
                )
            )
        credentials = "{}:{}".format(access_key, secret_key).encode("utf-8")
        if request_limit is not None and not 1 <= request_limit <= MAX_REQUEST_LIMIT:
            raise OracleError(
                "Onshape API request limit must be an integer from 1 through {}".format(
                    MAX_REQUEST_LIMIT
                )
            )
        self._authorization = "Basic " + base64.b64encode(credentials).decode("ascii")
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout
        self._request_limit = request_limit
        self._request_count = 0

    @classmethod
    def from_environment(cls):
        raw_limit = os.environ.get(REQUEST_LIMIT_ENV, "").strip()
        try:
            request_limit = int(raw_limit) if raw_limit else None
        except ValueError:
            raise OracleError("{} must be a positive integer".format(REQUEST_LIMIT_ENV))
        return cls(
            os.environ.get(ACCESS_KEY_ENV, ""),
            os.environ.get(SECRET_KEY_ENV, ""),
            os.environ.get(BASE_URL_ENV, DEFAULT_BASE_URL),
            request_limit=request_limit,
        )

    @property
    def request_count(self):
        """Number of host requests attempted by this transport."""
        return self._request_count

    @property
    def request_limit(self):
        """Configured host-request ceiling, or `None` for local manual use."""
        return self._request_limit

    def request(self, method, path, body=None, content_type=None):
        """Issue one request; returns `(status, response_bytes)`."""
        if self._request_limit is not None and self._request_count >= self._request_limit:
            raise OracleError(
                "Onshape API request limit exhausted ({}/{}) before {} request".format(
                    self._request_count, self._request_limit, method
                )
            )
        self._request_count += 1
        request = urllib.request.Request(self._base_url + path, data=body, method=method)
        request.add_header("Authorization", self._authorization)
        request.add_header("Accept", "application/json;charset=UTF-8; qs=0.09")
        if content_type is not None:
            request.add_header("Content-Type", content_type)
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()
        except urllib.error.URLError as error:
            raise OracleError("transport failure during {} request: {}".format(method, error))


def decode_json(status, body, action):
    """Decode a 2xx JSON response or raise with a bounded body excerpt."""
    if status < 200 or status >= 300:
        excerpt = body.decode("utf-8", errors="replace")[:200] if body else ""
        raise OracleError("{} failed: HTTP {} {}".format(action, status, excerpt))
    if not body:
        return {}
    try:
        return json.loads(body.decode("utf-8"))
    except ValueError:
        raise OracleError("{} returned non-JSON body".format(action))


def load_config(path=CONFIG_PATH):
    """Load and validate the untracked document-coordinates file."""
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise OracleError(
            "missing {}: run `python3 scripts/oracle_loop.py init` and fill in the "
            "document/workspace/element ids from the Onshape document URL".format(path)
        )
    except ValueError:
        raise OracleError("{} is not valid JSON".format(path))
    missing = [key for key in CONFIG_KEYS if not raw.get(key) or "<" in str(raw[key])]
    if missing:
        raise OracleError("{} is missing values for: {}".format(path, ", ".join(missing)))
    return {key: raw[key] for key in CONFIG_KEYS}


def session_info(transport):
    """Verify credentials; returns the authenticated session description."""
    status, body = transport.request("GET", "/api/v6/users/sessioninfo")
    return decode_json(status, body, "session check")


def upload_and_translate(transport, config, filename, payload):
    """Replace the blob element's content and return the started translation id.

    Onshape starts a Parasolid translation automatically on blob replacement.
    """
    content_type, body = encode_multipart("file", filename, payload)
    path = "/api/v6/blobelements/d/{document_id}/w/{workspace_id}/e/{element_id}".format(
        **config
    )
    status, response = transport.request("POST", path, body, content_type)
    data = decode_json(status, response, "blob upload of {}".format(filename))
    translation_id = data.get("translationId")
    if not translation_id:
        raise OracleError(
            "upload of {} succeeded but the response carried no translationId".format(filename)
        )
    return translation_id


def poll_translation(transport, translation_id, attempts=150, delay=2.0, sleep=time.sleep):
    """Poll one translation until it leaves ACTIVE; returns the terminal record."""
    for _ in range(attempts):
        status, body = transport.request("GET", "/api/v6/translations/" + translation_id)
        data = decode_json(status, body, "translation poll")
        if data.get("requestState") in TERMINAL_STATES:
            return data
        sleep(delay)
    raise OracleError(
        "translation {} still ACTIVE after {} polls".format(translation_id, attempts)
    )


@dataclass
class FixtureResult:
    """Terminal host verdict for one uploaded fixture."""

    name: str
    size: int
    state: str
    reason: str
    classification: str
    result_element_ids: list = field(default_factory=list)

    @property
    def accepted(self):
        return self.state == "DONE"


def run_fixture(transport, config, path, attempts=150, delay=2.0, sleep=time.sleep):
    """Upload one .x_t file and wait for the host verdict."""
    path = Path(path)
    payload = path.read_bytes()
    translation_id = upload_and_translate(transport, config, path.name, payload)
    terminal = poll_translation(transport, translation_id, attempts, delay, sleep)
    reason = terminal.get("failureReason") or ""
    return FixtureResult(
        name=path.name,
        size=len(payload),
        state=terminal.get("requestState", ""),
        reason=reason,
        classification=classify_failure(reason),
        result_element_ids=terminal.get("resultElementIds") or [],
    )


def list_elements(transport, config):
    """List the oracle document workspace's elements."""
    path = "/api/v6/documents/d/{document_id}/w/{workspace_id}/elements".format(**config)
    status, body = transport.request("GET", path)
    return decode_json(status, body, "element list")


def find_translated_part_studio(transport, config):
    """Locate the part studio the blob element translates into.

    Blob replacement re-translates into one derived part studio, and the
    translation record reports resultElementIds as null (live host, host
    version cloud-2026-07-11), so the studio is found by listing elements.
    The dedicated oracle document must therefore contain exactly one.
    """
    studios = [
        element
        for element in list_elements(transport, config)
        if element.get("elementType") == "PARTSTUDIO"
    ]
    if len(studios) != 1:
        raise OracleError(
            "expected exactly one part studio in the oracle document, found {}".format(
                len(studios)
            )
        )
    return studios[0]["id"]


def reexport_element(transport, config, element_id, attempts=12, delay=5.0, sleep=time.sleep):
    """Export a translated part studio back to Parasolid text synchronously.

    includeSurfaces=true is load-bearing: the default export silently drops
    sheet bodies and fails "No visible parts to export" (live host,
    cloud-2026-07-11). The same message also appears transiently because a
    DONE translation precedes the part studio's regeneration commit, so that
    specific rejection is retried before being treated as real.
    """
    path = (
        "/api/v6/partstudios/d/{}/w/{}/e/{}/parasolid"
        "?version=0&includeSurfaces=true".format(
            config["document_id"], config["workspace_id"], element_id
        )
    )
    for attempt in range(attempts):
        status, body = transport.request("GET", path)
        if 200 <= status < 300:
            return body
        excerpt = body.decode("utf-8", errors="replace")[:200] if body else ""
        if status == 400 and "No visible parts" in excerpt and attempt + 1 < attempts:
            sleep(delay)
            continue
        raise OracleError("re-export failed: HTTP {} {}".format(status, excerpt))
    raise OracleError("re-export produced no visible parts after {} attempts".format(attempts))
