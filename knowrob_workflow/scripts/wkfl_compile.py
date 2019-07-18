#project grasping.owl
#namespace derp derp.owl
#
#/*
#1 DeductiveIntegration[subject: ?s; predicate: ?p; constraint: ?c][object: ?o]
#2 PropertyRangeDeduction[subject:?s; predicate: ?p][propertyrange: ?pr]
#3 ClassRangeEnumeration[propertyrange: ?pr][objects: ?os]
#4 FirstNext[list: ?os][first: ?o; rest: ?os; have: ?t]
#5 Return[object: nil]
#6 ClassMembershipTest[object: ?o; class: ?c][result: ?t]
#7 Return[object: ?o]
#: 1->2
#: 2->3
#: 3->4
#: 4->5[?t=False]
#: 4->6[?t=True]
#: 6->4[?t=False]
#: 6->7[?t=True]
#*/

import os
import sys

inputFilePath = sys.argv[1]
outputOWLFilePath = sys.argv[2]
outputDOTFilePath = sys.argv[3]

def baseName(name, project):
    if -1 == name.find('#'):
        name = project + name
    return name

def unbaseName(name):
    split = name.find('#')
    if -1 != split:
        name = name[split + 1:]
    return name

def explicateEntity(name, namespaces):
    split = name.find('&')
    if -1 == split:
        return name
    ns = name[:split].strip()
    name = name[split + 1:].strip()
    if ns not in namespaces:
        print("WARNING: namespace %s not defined" % ns)
        return name
    return namespaces[ns] + name

def parseBindings(line, namespaces):
    warn = False
    split = line.find('[')
    split2 = line.find(']')
    if (-1 == split):
        if (-1 == split2):
            return [], "", False
        else:
            print("WARNING: unmatched ']':\n\t%s" % line)
            return [], "", True
    if -1 == split2:
        print("WARNING: unmatched '[':\n\t%s" % line)
        return [], "", True
    bstr = line[split + 1:split2].strip()
    bdgs = bstr.split(';')
    bindings = []
    for bdg in bdgs:
        bdg = bdg.strip()
        splitEq = bdg.find('=')
        splitCol = bdg.find(':')
        if -1 not in (splitEq, splitCol):
            print("WARNING: malformed binding uses both '=' and ':'\n\t" % line)
        if -1 != splitEq:
            bindings.append((explicateEntity(bdg[:splitEq].strip(), namespaces), "=", explicateEntity(bdg[splitEq + 1:].strip(), namespaces)))
        elif -1 != splitCol:
            bindings.append((explicateEntity(bdg[:splitCol].strip(), namespaces), ":", explicateEntity(bdg[splitCol + 1:].strip(), namespaces)))
    rest = line[split2 + 1:].strip()
    return bindings, rest, warn

def dotBindingsLabel(bindings):
    label = "["
    k = 0
    for bdg in bindings:
        k += 1
        label += unbaseName(bdg[0]) + bdg[1] + " " + unbaseName(bdg[2])
        if k < len(bindings):
            label += "; "
    label += "]"
    return label

def dotVertexLabel(vertex):
    label = vertex[0]
    split = label.find('#')
    if -1 != split:
        label = label[split + 1:]
    label = label + "\\n"
    label += dotBindingsLabel(vertex[1])
    if [] != vertex[2]:
        label += "\\n" + dotBindingsLabel(vertex[2])
    return label

def collectOWLInputsOutputs(definedTask, project):
    inputs = set([])
    outputs = set([])
    for bdg in definedTask[1]:
        rType = baseName(bdg[0], project)
        name = project + rType[rType.find('#') + 1:]
        inputs.add((rType, name))
    for bdg in definedTask[2]:
        rType = baseName(bdg[0], project)
        name = project + rType[rType.find('#') + 1:]
        outputs.add((rType, name))
    return inputs, outputs

def collectOWLVariables(verts, edges, project):
    variables = set([])
    for k, v in verts.items():
        for bdg in v[1] + v[2]:
            if ':' == bdg[1]:
                vName = baseName(bdg[2], project)
                variables.add(vName)
    for e in edges:
        for bdg in e[2]:
            vName = baseName(bdg[0], project)
            variables.add(vName)
            if ':' == bdg[1]:
                vName = baseName(vName, project)
                variables.add(vName)
    return variables

def collectOWLTasks(verts, entryPoints, exitPoints, project):
    tasks = set([project + "Entry", project + "Exit"])
    for k, v in verts.items():
        if (k not in entryPoints) and (k not in exitPoints):
            tasks.add(baseName(v[0], project))
    return tasks

def collectOWLInvocations(verts, entryPoints, exitPoints, project):
    invocations = {}
    numI = 0
    numB = 0
    for k, v in verts.items():
        numI = numI + 1
        name = project + "TaskInvocation_" + str(numI)
        taskName = v[0]
        if k == entryPoints[0]:
            taskName = project + "Entry"
        if k in exitPoints:
            taskName = project + "Exit"
        bindings = set([])
        inputBdgs = v[1]
        outputBdgs = v[2]
        if k == entryPoints[0]:
            inputBdgs = []
            outputBdgs = v[1]
        for bdg in inputBdgs:
            numB = numB + 1
            bName = project + "FactualBinding_" + str(numB)
            bType = bdg[1]
            role = baseName(bdg[0], project)
            filler = bdg[2]
            if ':' == bType:
                filler = baseName(filler, project)
            bindings.add((bName, role, bType, filler))
##### TODO: fix directions on the bindings for the Entry Task!!!! And check dir for the exit tasks!
        for bdg in outputBdgs:
            numB = numB + 1
            bName = project + "FactualBinding_" + str(numB)
            bType = bdg[1]
            role = baseName(bdg[2], project)
            filler = bdg[0]
            if ':' == bType:
                filler = baseName(filler, project)
            bindings.add((bName, role, bType, filler))
        invocations[k] = (name, taskName, frozenset(bindings))
    return invocations

def collectOWLSuccedences(edges, invocations, project):
    succedences = set([])
    numCB = 0
    numS = 0
    for e in edges:
        numS = numS + 1
        name = project + "ConditionalSuccedence_" + str(numS)
        cbindings = set([])
        predecessor = invocations[e[0]][0]
        successor = invocations[e[1]][0]
        bindings = set([])
        for bdg in e[2]:
            numCB = numCB + 1
            bName = project + "CounterfactualBinding_" + str(numCB)
            role = baseName(bdg[0], project)
            bType = bdg[1]
            filler = bdg[2]
            if ':' == bdg[1]:
                filler = baseName(filler, project)
            bindings.add((bName, role, bType, filler))
        succedences.add((name, predecessor, successor, frozenset(bindings)))
    return succedences

def writeOWLPreamble(outfile, project, namespaces):
    outfile.write('<?xml version="1.0"?>\n')
    outfile.write('<rdf:RDF xmlns="%s"\n' % project)
    outfile.write('     xml:base="%s"\n' % project[:-1])
    k = 0
    for ns, nsv in namespaces.items():
        k += 1
        outfile.write('     xmlns:%s="%s"' % (ns, nsv))
        if k == len(namespaces.keys()):
            outfile.write('>')
        outfile.write('\n')
    outfile.write('    <owl:Ontology rdf:about="%s">\n' % project[:-1])
    outfile.write('        <owl:imports rdf:resource="package://ease_ontologies/owl/EASE.owl"/>\n')
    outfile.write('        <owl:imports rdf:resource="http://www.ontologydesignpatterns.org/ont/dul/DUL.owl"/>\n')
    outfile.write('        <owl:imports rdf:resource="http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl"/>\n')
    for ns, nsv in namespaces.items():
        if ns not in fixed_ns:
            outfile.write('        <owl:imports rdf:resource="%s"/>\n' % nsv[:-1])
    outfile.write('    </owl:Ontology>\n\n\n')

def writeOWLSectionHeader(outfile, sectionHeader):
    outfile.write('    <!-- \n')
    outfile.write('    ///////////////////////////////////////////////////////////////////////////////////////\n')
    outfile.write('    //\n')
    outfile.write('    // %s\n' % sectionHeader)
    outfile.write('    //\n')
    outfile.write('    ///////////////////////////////////////////////////////////////////////////////////////\n')
    outfile.write('     -->\n\n\n')

def writeOWLVariables(outfile, variables):
    for v in variables:
        outfile.write('    <!-- %s -->\n\n' % v)
        outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % v)
        outfile.write('        <rdf:type>\n')
        outfile.write('            <owl:Class>\n')
        outfile.write('                <owl:unionOf rdf:parseType="Collection">\n')
        outfile.write('                    <rdf:Description rdf:about="http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Parameter"/>\n')
        outfile.write('                    <rdf:Description rdf:about="http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Role"/>\n')
        outfile.write('                </owl:unionOf>\n')
        outfile.write('            </owl:Class>\n')
        outfile.write('        </rdf:type>\n')
        outfile.write('    </owl:NamedIndividual>\n\n\n\n')

def writeOWLDefinedTask(outfile, definedTaskName, inputs, outputs, project):
    outfile.write('    <!-- %s -->\n\n' % definedTaskName)
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % definedTaskName)
    outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#OGPTask"/>\n')
    for inputRole in inputs:
        outfile.write('        <EASE-WF:definesInput rdf:resource="%s"/>\n' % inputRole[1])
    for outputRole in outputs:
        outfile.write('        <EASE-WF:definesOutput rdf:resource="%s"/>\n' % outputRole[1])
    outfile.write('    </owl:NamedIndividual>\n\n\n\n')
    outfile.write('    <!-- %s -->\n\n' % (project + "Entry"))
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % (project + "Entry"))
    outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#EntryOGPWorkflow"/>\n')
    for inputRole in inputs:
        outfile.write('        <EASE-WF:definesOutput rdf:resource="%s"/>\n' % inputRole[1])
    outfile.write('    </owl:NamedIndividual>\n\n\n\n')
    outfile.write('    <!-- %s -->\n\n' % (project + "Exit"))
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % (project + "Exit"))
    outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#ExitOGPWorkflow"/>\n')
    for outputRole in outputs:
        outfile.write('        <EASE-WF:definesInput rdf:resource="%s"/>\n' % outputRole[1])
    outfile.write('    </owl:NamedIndividual>\n\n\n\n')

def writeOWLBinding(outfile, bdg, project):
    bName = bdg[0]
    role = bdg[1]
    bType = bdg[2]
    filler = bdg[3]
    if ':' == bType:
        bType = "http://www.ease-crc.org/ont/EASE-WF.owl#RoleRoleBinding"
    else:
        bType = "http://www.ease-crc.org/ont/EASE-WF.owl#RoleFillerBinding"
    outfile.write('    <!-- %s -->\n\n' % bName)
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % bName)
    if -1 == bName.find("CounterfactualBinding_"):
        outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#FactualBinding"/>\n')
    else:
        outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#CounterfactualBinding"/>\n')
    outfile.write('        <rdf:type rdf:resource="%s"/>\n' % bType)
    outfile.write('        <EASE-WF:hasBindingFiller rdf:resource="%s"/>\n' % filler)
    outfile.write('        <EASE-WF:hasBindingRole rdf:resource="%s"/>\n'% role)
    outfile.write('    </owl:NamedIndividual>\n\n\n')

def writeOWLInvocation(outfile, invocation, project):
    name = invocation[0]
    taskName = baseName(invocation[1], project)
    bindings = invocation[2]
    outfile.write('    <!-- %s -->\n\n' % name)
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % name)
    outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#TaskInvocation"/>\n')
    outfile.write('        <DUL:definesTask rdf:resource="%s"/>\n' % taskName)
    for bdg in bindings:
        bName = bdg[0]
        outfile.write('        <EASE-WF:hasBinding rdf:resource="%s"/>\n' % bName)
    outfile.write('    </owl:NamedIndividual>\n\n\n\n')

def writeOWLInvocations(outfile, invocations, project):
    for k, v in invocations.items():
        for bdg in v[2]:
            writeOWLBinding(outfile, bdg, project)
        writeOWLInvocation(outfile, v, project)

def writeOWLSuccedence(outfile, succedence, project):
    name = succedence[0]
    predecessor = succedence[1]
    successor = succedence[2]
    bindings = succedence[3]
    outfile.write('    <!-- %s -->\n\n' % name)
    outfile.write('    <owl:NamedIndividual rdf:about="%s">\n' % name)
    outfile.write('        <rdf:type rdf:resource="http://www.ease-crc.org/ont/EASE-WF.owl#ConditionalSuccedence"/>\n')
    for bdg in bindings:
        bName = bdg[0]
        outfile.write('        <EASE-WF:hasBinding rdf:resource="%s"/>\n' % bName)
    outfile.write('        <EASE-WF:hasPredecessor rdf:resource="%s"/>\n' % predecessor)
    outfile.write('        <EASE-WF:hasSuccessor rdf:resource="%s"/>\n' % successor)
    outfile.write('    </owl:NamedIndividual>\n\n\n')


def writeOWLSuccedences(outfile, succedences, project):
    for s in succedences:
        for bdg in s[3]:
            writeOWLBinding(outfile, bdg, project)
        writeOWLSuccedence(outfile, s, project)

lines = open(inputFilePath).read().splitlines()

verts = {}
edges = []
project = ""
namespaces = {"rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#", "owl": "http://www.w3.org/2002/07/owl#", "xml": "http://www.w3.org/XML/1998/namespace", "swrl": "http://www.w3.org/2003/11/swrl#", "xsd": "http://www.w3.org/2001/XMLSchema#", "rdfs": "http://www.w3.org/2000/01/rdf-schema#", "DUL": "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#", "IOLite": "http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl#", "EASE": "http://www.ease-crc.org/ont/EASE.owl#", "EASE-WF": "http://www.ease-crc.org/ont/EASE-WF.owl#"}
fixed_ns = set(namespaces.keys())

seekProject = True
defineNamespace = True

k = 0
for line in lines:
    k = k + 1
    line = line.strip()
    if (0 == len(line)) or ('#' == line[0]):
        continue
    if seekProject:
        if 'project ' == line[:len('project ')]:
            seekProject = False
            project = line[len('project '):]
            if '#' != project[-1]:
                project += '#'
    else:
        if defineNamespace and ('namespace ' == line[:len('namespace ')]):
            line = line[len('namespace '):].strip()
            split = line.find(' ')
            if -1 != split:
                namespace_name = line[:split]
                namespace_value = line[split:].strip()
                if '#' != namespace_value[-1]:
                    namespace_value = namespace_value + '#'
                if namespace_name in fixed_ns:
                    print("WARNING: attempt to redefine fixed namespace %s, ignoring" % namespace_name)
                    continue
                namespaces[namespace_name] = namespace_value
        else:
            defineNamespace = False
            #: 6->7[?t=True]
            if ':' == line[0]:
                line = line[1:].strip()
                split = line.find('->')
                start = line[:split]
                rest = line[split + 2:]
                split = rest.find('[')
                end = rest
                if -1 != split:
                    end = rest[:split]
                rest = rest[split:]
                conditions, rest, warn = parseBindings(rest, namespaces)
                edges.append((start, end, conditions))
            else:
            #1 DeductiveIntegration[subject: ?s; predicate: ?p; constraint: ?c][object: ?o]
                split = line.find(' ')
                if -1 == split:
                    print("WARNING: malformed line %d" % k)
                    continue
                vertexId = line[:split].strip()
                line = line[split:].strip()
                split = line.find('[')
                if -1 == split:
                    print("WARNING: malformed line %d" % k)
                    continue
                taskType = explicateEntity(line[:split].strip(), namespaces)
                line = line[split:].strip()
                inBinds, line, warn = parseBindings(line, namespaces)
                outBinds, line, warn = parseBindings(line, namespaces)
                verts[vertexId] = (taskType, inBinds, outBinds)

#Sanity checks
endpoints = set([])
startpoints = set([])
for (s, e, cs) in edges:
    if e not in endpoints:
        endpoints.add(e)
    if s not in startpoints:
        startpoints.add(s)
    if (s not in verts) or (e not in verts):
        print("WARNING: undefined edge endpoint in pair (%s, %s)" % (s, e))

entryPoints = []
exitPoints = []
for k in verts.keys():
    if k not in endpoints:
        entryPoints.append(k)
    if k not in startpoints:
        exitPoints.append(k)

if 0 == len(entryPoints):
    print("WARNING: no entry point")
if 1 < len(entryPoints):
    print("WARNING: too many entry points")

print("VERTS-----------------")
for k, v in verts.items():
    print("%s: %s" % (k, str(v)))
print("EDGES-----------------")
for e in edges:
    print("%s" % str(e))
print("ENTRY-----------------")
print(entryPoints)
print("EXIT-----------------")
print(exitPoints)


#digraph G {
#  "entry" [xlabel="DeductiveIntegration\n(subject:?s; predicate: ?p; constraint: ?c)\n(object: ?o)", shape=circle, style=filled, width=0.1, fixedsize=true, fillcolor=black, label=""]
#  "prd" [label="PropertyRangeDeduction\n(subject:?s; predicate: ?p)\n(propertyrange: ?pr)", shape=box]
#  "cre" [label="ClassRangeEnumeration\n(propertyrange: ?pr)\n(objects: ?os)", shape=box]
#  "fn" [label="FirstNext\n(list: ?os)\n(first: ?o; rest: ?os; have: ?t)", shape=box]
#  "ext_err" [xlabel ="Return\n(object: nil)", shape=circle, style=filled, width=0.1, fixedsize=true, fillcolor=black, label=""]
#  "cmt" [label="ClassMembershipTest\n(object: ?o; class: ?c)\n(result: ?t)", shape=box]
#  "ext_ok" [xlabel="Return\n(object: ?o)", shape=circle, style=filled, width=0.1, fixedsize=true, fillcolor=black, label=""]
#  "entry" -> "prd"
#  "prd" -> "cre"
#  "cre" -> "fn"
#  "fn" -> "ext_err" [label="?t: False"]
#  "fn" -> "cmt" [label="?t: True"]
#  "cmt" -> "fn" [label="?t: False"]
#  "cmt" -> "ext_ok" [label="?t: True"]
#}
with open(outputDOTFilePath, "w") as outfile:
    outfile.write("digraph G {\n")
    verts_sorted = []
    for k, v in verts.items():
        verts_sorted.append((k, v))
    verts_sorted.sort(key=lambda(x): x[0])
    for (k, v) in verts_sorted:
        label = ""
        taskDesc = dotVertexLabel(v)
        if k in entryPoints:
            label = "[xlabel=\"%s\", shape=circle, style=filled, width=0.1, fixedsize=true, fillcolor=black, label=\"\"]" % taskDesc
        elif k in exitPoints:
            label = "[xlabel=\"%s\", shape=circle, style=filled, width=0.1, fixedsize=true, fillcolor=black, label=\"\"]" % taskDesc
        else:
            label = "[label=\"%s\", shape=box]" % taskDesc
        outfile.write("\"%s\" %s\n" % (k, label))
    for edge in edges:
        cstr = dotBindingsLabel(edge[2])
        if ("" != cstr) and ("[]" != cstr):
            cstr = " [label=\"%s\"]" % cstr
        else:
            cstr = ""
        outfile.write("\"%s\" -> \"%s\"%s\n" % (edge[0], edge[1], cstr))
    outfile.write("}\n")

with open(outputOWLFilePath, "w") as outfile:
    writeOWLPreamble(outfile, project, namespaces)
    writeOWLSectionHeader(outfile, "Individuals")
    definedTask = verts[entryPoints[0]]
    definedTaskName = baseName(definedTask[0], project)
    inputs, outputs = collectOWLInputsOutputs(definedTask, project)
    variables = collectOWLVariables(verts, edges, project)
    tasks = collectOWLTasks(verts, entryPoints, exitPoints, project)
    invocations = collectOWLInvocations(verts, entryPoints, exitPoints, project)
    succedences = collectOWLSuccedences(edges, invocations, project)
    writeOWLVariables(outfile, variables)
    writeOWLDefinedTask(outfile, definedTaskName, inputs, outputs, project)
    writeOWLInvocations(outfile, invocations, project)
    writeOWLSuccedences(outfile, succedences, project)

    outfile.write('    <!-- %sWorkflowIndividual -->\n\n' % project)
    outfile.write('    <owl:NamedIndividual rdf:about="%sWorkflowIndividual">\n' % project)
    outfile.write('        <rdf:type rdf:resource="%sOGPWorkflow"/>\n' % namespaces['EASE-WF'])
    outfile.write('        <EASE-WF:isWorkflowFor rdf:resource="%s"/>\n' % definedTaskName)
    for v in variables:
        outfile.write('        <DUL:defines rdf:resource="%s"/>\n' % v)
    for t in tasks:
        outfile.write('        <DUL:definesTask rdf:resource="%s"/>\n' % t)
    for k, v in invocations.items():
        outfile.write('        <DUL:defines rdf:resource="%s"/>\n' % v[0])
    for s in succedences:
        outfile.write('        <EASE-WF:hasSuccedence rdf:resource="%s"/>\n' % s[0])
    outfile.write('    </owl:NamedIndividual>\n\n\n\n')

    writeOWLSectionHeader(outfile, "Classes")
    for role in inputs|outputs:
        if 0 == role[0].find(project):
            outfile.write('\n')
            outfile.write('    <!-- %s -->\n\n' % role[0])
            outfile.write('    <owl:Class rdf:about="%s">\n' % role[0])
            outfile.write('        <rdfs:subClassOf>\n')
            outfile.write('            <owl:Class>\n')
            outfile.write('                <owl:unionOf rdf:parseType="Collection">\n')
            outfile.write('                    <rdf:Description rdf:about="http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Parameter"/>\n')
            outfile.write('                    <rdf:Description rdf:about="http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Role"/>\n')
            outfile.write('                </owl:unionOf>\n')
            outfile.write('            </owl:Class>\n')
            outfile.write('        </rdfs:subClassOf>\n')
            outfile.write('    </owl:Class>\n\n\n')
    outfile.write("</rdf:RDF>\n")

