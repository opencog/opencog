/** 
 * LearningServer.h
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef LS_H
#define LS_H

#include "comboreduct/combo/vertex.h"

#include "util/Logger.h"

#include "WorldProvider.h"
#include "NetworkElement.h"
#include "SystemParameters.h"

#include "LSCmdMessage.h"
#include "LearnMessage.h"
#include "RewardMessage.h"
#include "ImitationLearningTask.h"
#include "TrySchemaMessage.h"
#include "StopLearningMessage.h"

namespace LearningServer{

class LS : public MessagingSystem::NetworkElement {
	
	public:
		
		/**
		 * Constructor and Destructor
		 */
		LS(const std::string &myId, const std::string &ip, int portNumber, 
		   Control::SystemParameters & parameters);
		~LS();
	
		/** 
		 * Method inherited from network element 
		 */
		bool processNextMessage(MessagingSystem::Message *msg);		
	
		/** 
		 * Method inherited from network element 
		 */
		void setUp();
		
		/**
		 * Informs whenever the LS is busy performing a learning action or not.
		 * 
		 * @return True if LS is busy, false otherwise.
		 */
		bool isBusy();
		
		/**
		 * Try a candidate schema of the schema being learned. This schema is 
		 * send to OPC in order to be evaluated by the pet owner. In order to 
		 * send a candidate schema, the learning algorithm is pause and waits
		 * for a feedback from the user (a reward message)
		 * 
		 * @param schema The combo combo_tree representation of the schema 
		 */
		void sendCandidateSchema(const combo::combo_tree & schema);
		
		/**
		 * Send the best schema so far to be stored in the OPC procedure repository
		 * 
		 * @param schema The combo combo_tree representation of the schema 
		 */
		void sendBestSchema(const combo::combo_tree & schema);
	
	private:	
		
		WorldProvider *wp;			// store behavior descriptors and space server
		                                        // with latest map
		
		bool busy;						// LS busy learning trick?

		std::string learningPet;		// the id of the pet using the LS
		std::string ownerID;                    // the id of the owner of the pet
		std::string avatarID;                   // the id of the avatar to imitate
		std::string learningSchema;             // the trick being learned
		
		int candidateSchemaCnt;

		//imitation learning task (can plug hillclimbing or MOSES)
		MessagingSystem::ImitationLearningTask ILTask;
		
		/**
		 * Return the candidate schema name according to the acctual candidate 
		 * schema count.
		 * 
		 * @return A std::string with the candidate schema name 
		 */
		const std::string getCandidateSchemaName();
		
		/**
		 * Set the learning algorithm environment with data passed within the 
		 * learn message (schema to learn, world map, example actions)
		 * 
		 * @param msg Learning message with enviroment data
		 */
		void initLearn(LearningServerMessages::LearnMessage * msg);
		
		/**
		 * Add new data into the learning algorithm enviroment. This data represents
		 * new schema examples and new world map data. This data is merged with 
		 * previous received data.
		 * 
		 * @param msg Learning message with enviroment data
		 */
		void addLearnExample(LearningServerMessages::LearnMessage * msg);
		
		/**
		 * Insert user feedback into the learning process (via RewardMessage). 
		 * 
		 * @param msg A RewardMessage with user feedback about a candidate schema 
		 */
		void rewardCandidateSchema(LearningServerMessages::RewardMessage * msg);
		
		/**
		 * Clear all LS structures used during learning process.
		 */
		void resetLearningServer();
		
		/**
		 * Encapsulate the combo combo_tree representation of the learned schema, its
		 * name and possible its candidate name (if trying a candidate schema).
		 * 
		 * @param schema A combo combo_tree representation of the schema
		 * @param schemaName The name of the schema
		 * @param candidateName The name of the candidate schema of the schema 
		 * 						being learned (optional).
		 */ 
		void sendSchema(const combo::combo_tree & schema, 
						std::string schemaName, 
						std::string candidateName = "");
		
		/**
		 * Finishs the learning process, get the best schema so far and send it to 
		 * the OPC to be added into pet's procedure repository.
		 */
		void stopLearn();
		
		/**
		 * Get the best schema so far during learning process and send it to 
		 * the OPC in order to be evaluated by the owner of the pet. Note: the
		 * learning process is paused when a candidate schema is sent to evaluation.
		 */
		void trySchema();
		
				
}; // class
}  // namespace
 
#endif 
