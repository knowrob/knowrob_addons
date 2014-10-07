package org.knowrob.constr.util;

import java.util.HashSet;
import java.util.Set;

import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectSomeValuesFrom;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.util.OWLClassExpressionVisitorAdapter;

/** Visits existential restrictions and collects the properties which are restricted
 * 
 *  (adapted from OWLAPI examples)
 * 
 */


public class RestrictionVisitor  extends OWLClassExpressionVisitorAdapter {

	private Set<OWLClass> processedClasses;
//	private Set<OWLObjectPropertyExpression> restrictedProperties;
	private Set<OWLClassExpression> restrictionFillers;
	private Set<OWLOntology> onts;
	private OWLObjectProperty prop;

	public RestrictionVisitor(Set<OWLOntology> onts, OWLObjectProperty prop) {
		restrictionFillers = new HashSet<OWLClassExpression>();
		processedClasses = new HashSet<OWLClass>();
		this.onts = onts;
		this.prop = prop;
	}

	public Set<OWLClassExpression> getRestrictionFillers() {
		return restrictionFillers;
	}

	@Override
	public void visit(OWLClass desc) {

		if (!processedClasses.contains(desc)) {
			
			processedClasses.add(desc);
			
			for (OWLOntology ont : onts) {
				for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(desc)) {
					ax.getSuperClass().accept(this);
				}
			}
		}
	}

	@Override
	public void visit(OWLObjectSomeValuesFrom desc) {
		// This method gets called when a class expression is an existential
		// (someValuesFrom) restriction and it asks us to visit it

		if(desc.getProperty().equals(prop))
			restrictionFillers.add(desc.getFiller());
	}
} 