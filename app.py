import os
import webbrowser
from flask import Flask, escape, request, render_template

template_dir = os.path.join("flask", "html")
static_dir = os.path.join("/flask")
app = Flask(
    __name__,
    template_folder=template_dir,
    static_url_path=static_dir,
    static_folder='flask'
)


@app.route('/')
def hello():
    name = request.args.get("name", "World")
    return render_template('hello.html', name=name)


@app.route('/coppelia')
def coppelia():
    return f'Here is the Coppelia page'


@app.route('/test/<int:my_int>')
def test(my_int):
    return f'Here is the test page - num: {my_int}'


@app.route('/viz')
def viz():
    return render_template('viz_development.html')


@app.route('/vizEx1')
def viz_example1():
    return render_template('viz_example-1.html')


# webbrowser.open("http://localhost:5000/?name=Drew")
# webbrowser.open("http://localhost:5000/vizEx1")
webbrowser.open("http://localhost:5000/viz")


if __name__ == '__main__':
    app.run()
