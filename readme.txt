source env/bin/activate
# Activar el env 
env\Scripts\Activate.ps1
# Correr FastaPI
uvicorn main:app --reload