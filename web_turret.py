import flask
import threading
import turret_with_config
import cv2
import requests
import time

app = flask.Flask(__name__)
turret = None

@app.before_first_request
def on_web_start():
    global turret, th, green_image
    def thread():
        global turret
        turret = turret_with_config.start(turret)
    green_image = False
    turret = turret_with_config.create()
    th = threading.Thread(target=thread).start()

@app.route("/")
def index():
    return flask.send_from_directory('web_turret', "index.html")

@app.route("/ms")
def turret_img():
    global turret, green_image
    img = turret.motion_sensor.last_image
    if img is None:
        return flask.send_from_directory('web_turret', 'no_image.png')
    if green_image:
        #img[:,:,1] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img[:,:,0] = 0
        img[:,:,2] = 0
    _, png = cv2.imencode(".png", img)
    response = flask.make_response(png.tobytes())
    response.headers["Content-Type"] = "image/png"
    return response

@app.route("/ping")
def ping():
    return "PING"

@app.route("/turret_info")
def turret_info():
    global turret
    return flask.render_template("turret_info.html", turret=turret)

@app.route("/change_friendly")
def change_friendly():
    state = flask.request.args.get("state") == "true"
    turret.gun.friendly = state
    return ""

@app.route("/change_show_video")
def change_show_video():
    state = flask.request.args.get("state") == "true"
    turret.motion_sensor.show_video = state
    cv2.destroyAllWindows()
    return ""

@app.route("/green_image")
def green_image():
    global green_image
    green_image = flask.request.args.get("state") == "true"
    return ""

@app.route("/image_width")
def image_width():
    value = int(flask.request.args.get("value"))
    if value >= 50 and value <= 2000:
        turret.motion_sensor.image_width = value
    return ""

@app.route("/frame_rate")
def frame_rate():
    value = 1 / float(flask.request.args.get("value"))
    if value >= 0.1 and value <= 2.5:
        turret.motion_sensor.max_frame_rate = value
    return ""

@app.route("/blur_radius")
def blur_radius():
    value = int(flask.request.args.get("value"))
    if value >= 0 and value <= 50:
        turret.motion_sensor.blur_radius = value*2+1
    return ""

@app.route("/img_threshold")
def img_threshold():
    value = int(flask.request.args.get("value"))
    if value >= 1 and value <= 200:
        turret.motion_sensor.threshold = value
    return ""

@app.route("/static_count_limit_start")
def static_count_limit_start():
    value = int(flask.request.args.get("value"))
    if value >= 2 and value <= 200:
        turret.motion_sensor.static_count_limit_start = value
    return ""

@app.route("/static_count_limit_live")
def static_count_limit_live():
    value = int(flask.request.args.get("value"))
    if value >= 2 and value <= 200:
        turret.motion_sensor.static_count_limit_live = value
    return ""

@app.route("/override")
def override():
    if (flask.request.args.get("x") == "OFF"):
        turret.override_motion = False
    else:
        x = int(flask.request.args.get("x")) / 100
        y = int(flask.request.args.get("y")) / 100
        turret.override_target(x, y)
    return ""

def start_runner():
    # Ping webserver in separate thread so that on_web_start is called
    def thread():
        not_started = True
        while not_started:
            try:
                r = requests.get("http://127.0.0.1:8080/ping")
                if r.status_code == 200:
                    not_started = False
                    print("Web server response received.")
            except:
                print("Waiting for web server")
            time.sleep(1)

    print("PINGING web server")
    t = threading.Thread(target=thread, daemon=True)
    t.start()

if __name__ == "__main__":
    start_runner()
    app.run(host="0.0.0.0", port=8080, debug=True)
