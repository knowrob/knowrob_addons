/*
 * Copyright (c) 2012-14 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
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