using IJulia
jupyter = IJulia.jupyter
notebook = "../demo.ipynb"
run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
