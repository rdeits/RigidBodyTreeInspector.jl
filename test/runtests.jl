using IJulia
jupyter = IJulia.jupyter
notebook = "../examples/demo.ipynb"
run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
