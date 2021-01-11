import os
import webbrowser
from flask import Flask, escape, request, render_template

template_dir = os.path.join("flask", "html")
app = Flask(__name__, template_folder=template_dir)


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


webbrowser.open("http://localhost:5000/?name=Drew")


if __name__ == '__main__':
    app.run()
