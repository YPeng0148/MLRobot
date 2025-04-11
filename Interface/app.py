from flask import Flask, request, render_template
from trial import direction

app = Flask(__name__, template_folder='html/', static_url_path='/static')
map = {"Hajim Statue": "1", "Design Demo": "2",
       "Wegmans Hall": "3", "Hopeman Gear": "4"}


@app.route('/', methods=['GET', 'POST'])
def choose_option():
    if request.method == 'POST':
        # handling error
        if 'option' not in request.form:
            flag = True
            return render_template('index.html', flag=flag)
        else:
            name = request.form['option']
            var = map[name]
            direction(var)
            return render_template('thanks.html', name=name)
    else:
        return render_template('index.html')


if __name__ == '__main__':
    app.run(debug=True)
