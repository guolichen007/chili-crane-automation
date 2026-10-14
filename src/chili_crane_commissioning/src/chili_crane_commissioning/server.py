"""Local HTTP interface: bounded input, same-origin token, no arbitrary commands."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import secrets
from urllib.parse import urlsplit


class ConsoleServer(ThreadingHTTPServer):
    def __init__(self,address,backend,asset_directory=None,allow_nonlocal=False,allowed_hosts=()):
        host,port=address
        loopback=host=='localhost'
        try:loopback=loopback or ipaddress.ip_address(host).is_loopback
        except ValueError:pass
        if not loopback and not allow_nonlocal:raise PermissionError('EXPLICIT_NONLOCAL_LISTEN_OPTION_REQUIRED')
        if not loopback and backend.bench.enabled:raise PermissionError('ARMED_BENCH_LOOPBACK_ONLY')
        self.backend=backend
        self.allowed_hosts={'localhost','127.0.0.1','::1',host}|set(allowed_hosts)
        assets=Path(asset_directory) if asset_directory else Path(__file__).resolve().parent/'web'
        if not assets.exists():
            from ament_index_python.packages import get_package_share_directory
            assets=Path(get_package_share_directory('chili_crane_commissioning'))/'web'
        self.assets=assets
        super().__init__(address,ConsoleHandler)


class ConsoleHandler(BaseHTTPRequestHandler):
    def log_message(self,format,*args):pass

    def reply(self,status,value,content_type='application/json; charset=utf-8'):
        raw=value if isinstance(value,bytes) else json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8')
        self.send_response(status);self.send_header('Content-Type',content_type)
        self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'")
        self.end_headers();self.wfile.write(raw)

    def origin(self):
        host=self.headers.get('Host','')
        parsed=urlsplit('http://'+host)
        if parsed.hostname not in self.server.allowed_hosts or parsed.username or parsed.password:
            raise PermissionError('HOST_NOT_ALLOWED')
        origin=self.headers.get('Origin')
        if origin is not None:
            actual=urlsplit(origin)
            if actual.scheme!='http' or actual.netloc!=host:raise PermissionError('CROSS_ORIGIN_FORBIDDEN')

    def do_GET(self):
        try:
            self.origin();path=urlsplit(self.path).path
            if path=='/api/state':self.reply(200,self.server.backend.state());return
            if path.startswith('/api/records/'):
                raw=self.server.backend.recorder.read(path.rsplit('/',1)[1])
                self.reply(200,raw,'application/x-ndjson; charset=utf-8');return
            names={'/':'index.html','/app.js':'app.js','/style.css':'style.css'}
            if path not in names:self.reply(404,{'error':'NOT_FOUND'});return
            content={'/':'text/html','/app.js':'text/javascript','/style.css':'text/css'}[path]+'; charset=utf-8'
            self.reply(200,(self.server.assets/names[path]).read_bytes(),content)
        except PermissionError as exc:self.reply(403,{'error':str(exc)})
        except (OSError,ValueError) as exc:self.reply(400,{'error':str(exc)})

    def do_POST(self):
        try:
            self.origin();backend=self.server.backend
            token=self.headers.get('X-CSRF-Token','')
            if not secrets.compare_digest(token,backend.token):raise PermissionError('SESSION_TOKEN_REQUIRED')
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=65536:raise ValueError('REQUEST_BODY_BOUND')
            body=json.loads(self.rfile.read(length).decode('utf-8'))
            if not isinstance(body,dict):raise ValueError('JSON_OBJECT_REQUIRED')
            path=urlsplit(self.path).path
            if path=='/api/simulation':result=backend.remote(body.get('signal'),body.get('active',False))
            elif path=='/api/stop':result=backend.release('UI_STOP_RELEASE_BLUR_OR_DISCONNECT')
            elif path=='/api/bench/pulse':result=backend.bench.pulse(body.get('signal'),body.get('duration_ms',500))
            elif path=='/api/bench/stop':backend.bench.stop.set();result={'reason':'OWNED_BENCH_PULSE_CANCEL_REQUESTED_NOT_OFF_PROOF'}
            elif path=='/api/bench/voltage':result=backend.voltage(body)
            elif path=='/api/recording/start':result=backend.bag.start(body.get('profile','state_only'))
            elif path=='/api/recording/stop':result=backend.bag.stop()
            else:self.reply(404,{'error':'NOT_FOUND'});return
            self.reply(200,result)
        except PermissionError as exc:self.reply(403,{'error':str(exc)})
        except (ValueError,KeyError,TypeError) as exc:self.reply(400,{'error':str(exc)})
        except Exception as exc:self.reply(500,{'error':str(exc),'reason':'FAILED_NO_FIELD_CLAIM'})
