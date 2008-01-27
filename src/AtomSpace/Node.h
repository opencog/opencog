/**
 * Node.h
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef CORE_NODE_H
#define CORE_NODE_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "Atom.h"
//#include "types.h"

class Atom;

/**
 * This is a subclass of Atom. It represents the most basic kind of pattern
 * known to the OpenCog system.
 */

class Node : public Atom {

    friend class SavingLoading;

    private:

        // properties
        std::string name;

    public:

        /**
         * Constructor for this class.
         *
         * @param Node type
         * @param Node name A reference to a std::string with the name of the node. 
         *                  Use empty string for unamed node.
         * @param Node truthvalue A reference to a TruthValue object. 
         */
        Node(Type, const std::string&, const TruthValue& = TruthValue::NULL_TV())
             throw (InvalidParamException, AssertionException);
        
        /**
         * Destructor for this class.
         */
        virtual ~Node() throw ();

        /**
         * Gets the name of the node.
         *
         * @return The name of the node.
         */
        const std::string& getName() const;

        /*
         * @param Node name A reference to a std::string with the name of the node. 
         *                  Use empty string for unamed node.
         * @exception RuntimeException is thrown if this method is called for an Node already 
         *                  inserted into AtomSpace. Otherwise, internal index structures 
         *                  would become inconsistent.
         */
        void  setName(const std::string&) throw (RuntimeException);

        /**
         * Merges two nodes.
         *
         * @param Node to be merged.
         */
        virtual void merge(Atom*) throw (InconsistenceException);

        /**
         * Returns a string representation of the node. 
         *
         * @return A string representation of the node.
         */
        std::string toString();
        std::string toShortString();

        /**
         * Returns whether a given atom is equal to the current node.
         * @param Node to be tested.
         * @return true if they are equal, false otherwise.
         */
        virtual bool equals(Atom*);

        /**
        * Returns the hashCode of the Node.
        * @return a integer value as the hashCode of the Node.
        */
        virtual int hashCode();
};

#endif
