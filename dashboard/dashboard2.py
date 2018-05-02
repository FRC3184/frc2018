import json
from queue import Queue
from threading import Thread

from wpilib.command import Command

from control import pose_estimator, robot_time
from dashboard import dashboard_server
from dashboard.dashboard_server import FileResponse, GeneratorResponse, ServerSentEvent

subscriptions = []
graphs = {}
choosers = {}
indicators = {}
number_inputs = {}
chooser_status = {}

extensions = {}


class ExtensionJSResponse(dashboard_server.Response):
    def respond(self, handler, path):
        self.begin(handler)
        for ex_name, ex in extensions.items():
            handler.wfile.write(ex.js.encode("U8"))
        self.end(handler)


class ExtensionCSSResponse(dashboard_server.Response):
    def respond(self, handler, path):
        self.begin(handler)
        for ex_name, ex in extensions.items():
            handler.wfile.write(ex.css.encode("U8"))
        self.end(handler)

dashboard_server.serve_path("/", FileResponse("dashboard/index.html"))
dashboard_server.serve_directory("js", path="dashboard/js", mimetype="application/javascript")
dashboard_server.serve_directory("images", path="dashboard/images", mimetype="image/png")
dashboard_server.serve_directory("css", path="dashboard/css", mimetype="text/css")
dashboard_server.serve_path("/extensions.js", ExtensionJSResponse(mimetype="application/javascript"))
dashboard_server.serve_path("/extensions.css", ExtensionJSResponse(mimetype="text/css"))


class Extension:
    def __init__(self, name, html, js, css):
        self.name = name
        self.html = html
        self.js = js
        self.css = css


def extension(name, html, js, css, callback_path=None, callback=None):
    ex = Extension(name, html, js, css)
    extensions[name] = ex

    if callback_path is not None and callable(callback):
        dashboard_server.method_path(callback_path, callback)


def run(basedir):
    server_thread = Thread(target=lambda: dashboard_server.run(basedir=basedir))
    server_thread.daemon = True
    server_thread.start()


def gen():
    q = Queue()
    for g, _ in graphs.items():
        q.put(ServerSentEvent(json.dumps({"action": "make_chart", "name": g}), "action"))
    for name, chooser in choosers.items():
        opts = list(chooser.options.keys())
        q.put(ServerSentEvent(json.dumps({"action": "make_chooser", "name": name, "options": opts}), "action"))
    for name, _ in indicators.items():
        q.put(ServerSentEvent(json.dumps({"action": "make_indicator", "name": name}), "action"))
    for name, obj in extensions.items():
        q.put(ServerSentEvent(json.dumps({"action": "make_extension", "name": name, "html": obj.html}), "action"))
    for name, value in number_inputs.items():
        q.put(ServerSentEvent(json.dumps({"action": "make_input", "name": name, "value": value}), "action"))

    subscriptions.append(q)
    try:
        while True:
            ev = q.get()
            yield ev.encode()
    except GeneratorExit:
        subscriptions.remove(q)
        print("Dropped a stream")


dashboard_server.serve_path("/events", GeneratorResponse(gen))


def update_chooser(handler, path, data):
    data = json.loads(data.decode("U8"))
    opt = data['option']
    chooser = choosers[data['name']]
    chooser.selected = (None if opt == "<Select>" else opt)
    return 200


dashboard_server.method_path("/update_chooser", update_chooser)


def update_input(handler, path, data):
    data = json.loads(data.decode("U8"))
    opt = data['value']
    number_inputs[data['name']] = opt
    print("Received number for {}: {}".format(data['name'], opt))
    return 200


dashboard_server.method_path("/update_number", update_input)


def add_graph(name, callback):
    graphs[name] = callback


class Chooser:
    def __init__(self, name):
        self.name = name
        self.options = {}
        self.selected = None
        self.default = None

    def add_option(self, name, object):
        self.options[name] = object

    def get_selected(self):
        return self.options[self.selected] if self.selected is not None else self.default

    def set_default(self, name):
        self.default = self.options[name]


def add_chooser(name) -> Chooser:
    choosers[name] = Chooser(name)
    return choosers[name]


def add_indicator(name, callback):
    indicators[name] = callback


def number_input(name, default=0):
    number_inputs[name] = default


def update(time):
    for g, v in graphs.items():
        send_message({"name": g, "time": time, "value": v()}, event="data")
    for name, status in indicators.items():
        send_message({"name": name, "status": status()}, event='indicator')
    # Try to send the pose, but it will throw an error if it hasn't been initialized yet
    try:
        poze = pose_estimator.get_current_pose()
        send_message({"x": poze.x * 12, "y": -poze.y * 12, "heading": poze.heading}, event="pose")
    except ValueError:
        pass


def send_message(data, event=None):
    sse = ServerSentEvent(json.dumps(data), event=event)
    for qq in subscriptions:
        qq.put(sse)


class DashboardUpdateCommand(Command):
    def __init__(self):
        super().__init__(name="dashboard2 update")
        self.setRunWhenDisabled(True)

    def initialize(self):
        pass

    def execute(self):
        update(robot_time.delta_time() / 1000)

    def isFinished(self):
        return False