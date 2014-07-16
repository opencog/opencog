# Installs the dependencies for the REST API
# Usage:
#   sudo ./install_dependencies.sh
#
# Todo: After https://github.com/opencog/opencog/issues/337 is addressed, these dependencies will be
#       handled automatically using requirements.txt

# Required:
pip install Flask
pip install -U mock
pip install -U flask-restful
pip install flask-restful-swagger
pip install six
pip install graphviz

# Optional:
pip install requests
