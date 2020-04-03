## Workflow interpreter
An interpreter for an OWL-representation of workflows.

# Workflow compilation script

To use ./scripts/wkfl_compile.py, call it as in the following example:

```
mkdir ./output
python ./scripts/wkfl_compile ./example/putpartstogether.wkf ./output/putpartstogether.owl ./output/putpartstogether.dot
```

The script takes three arguments, which are, in order, a path to the input workflow file (which must exist), a path to the output owl file, and a path to the output dot file.

Dot files can be converted to a pdf to display the graph they describe similar to the following example

```
dot -Tpdf ./output/putpartstogether.dot -o ./output/putpartstogether.pdf
```

