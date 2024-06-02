import os
from flask import Flask, send_file

# Correct the path to the static folder
static_folder = os.path.join(os.path.dirname(__file__), 'static')
index_file = os.path.join(static_folder, 'index.html')
print(f"Static folder path: {static_folder}")  # Debug print
print(f"Index file path: {index_file}")  # Debug print

app = Flask(__name__, static_folder=static_folder)

@app.route('/')
def index():
    if os.path.exists(index_file):
        print("Serving index.html")  # Debug print
        return send_file(index_file)
    else:
        print("index.html not found")  # Debug print
        return "index.html not found", 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)

