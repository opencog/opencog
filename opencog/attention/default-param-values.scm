(define AF_RENT_FREQUENCY  (ConceptNode "ECAN_AF_RENT_FREQUENCY"))
(define MAX_AF_SIZE (ConceptNode "MAX_AF_SIZE"))
(define AF_SIZE (ConceptNode "ECAN_AF_SIZE"))
(define AFB_BOTTOM  (ConceptNode "ECAN_AFB_BOTTOM"))
(define AFB_DECAY (ConceptNode "ECAN_AFB_DECAY"))
(define AFB_SIZE (ConceptNode "ECAN_AFB_SIZE"))
(define ECAN_PARAM (ConceptNode "ECAN_PARAMS"))
(define FORGET_THRESHOLD (ConceptNode "ECAN_FORGET_THRESHOLD") ) 
(define MAX_LINKS (ConceptNode "ECAN_MAX_LINKS")   ) 
(define HEBBIAN_MAX_ALLOCATION_PERCENTAGE (ConceptNode "HEBBIAN_MAX_ALLOCATION_PERCENTAGE") )  
(define LOCAL_FAR_LINK_RATIO (ConceptNode "ECAN_LOCAL_FAR_LINK_RATIO") )
(define MAX_SPREAD_PERCENTAGE  (ConceptNode "ECAN_MAX_SPREAD_PERCENTAGE"))
(define SPREAD_HEBBIAN_ONLY (ConceptNode "ECAN_SPREAD_HEBBIAN_ONLY"))
(define DIFFUSION_TOURNAMENT_SIZE(ConceptNode "ECAN_DIFFUSION_TOURNAMENT_SIZE"))
(define STARTING_ATOM_STI_RENT (ConceptNode "ECAN_STARTING_ATOM_STI_RENT"))
(define STARTING_ATOM_LTI_RENT (ConceptNode "ECAN_STARTING_ATOM_LTI_RENT"))
(define TARGET_STI_FUNDS (ConceptNode "TARGET_STI_FUNDS"))
(define STI_FUNDS_BUFFER (ConceptNode "STI_FUNDS_BUFFER"))
(define LTI_FUNDS_BUFFER (ConceptNode "LTI_FUNDS_BUFFER"))
(define TARGET_LTI_FUNDS_BUFFER (ConceptNode "TARGET_LTI_FUNDS_BUFFER"))
(define RENT_TOURNAMENT_SIZE (ConceptNode "ECAN_RENT_TOURNAMENT_SIZE"))

(MemberLink 
  AFB_SIZE
  ECAN_PARAM  
)
(MemberLink 
  AFB_DECAY
  ECAN_PARAM  
)
(MemberLink 
  AFB_BOTTOM
  ECAN_PARAM  
)
(MemberLink 
  AF_SIZE
  ECAN_PARAM  
)
(MemberLink 
  MAX_AF_SIZE
  ECAN_PARAM  
)
(MemberLink 
  AF_RENT_FREQUENCY 
  ECAN_PARAM  
)
(MemberLink 
  FORGET_THRESHOLD
  ECAN_PARAM  
)
(MemberLink 
  MAX_LINKS
  ECAN_PARAM  
)
(MemberLink 
  HEBBIAN_MAX_ALLOCATION_PERCENTAGE
  ECAN_PARAM  
)
(MemberLink 
  LOCAL_FAR_LINK_RATIO
  ECAN_PARAM  
)
(MemberLink 
  MAX_SPREAD_PERCENTAGE
  ECAN_PARAM  
)
(MemberLink 
  SPREAD_HEBBIAN_ONLY
  ECAN_PARAM  
)
(MemberLink 
  DIFFUSION_TOURNAMENT_SIZE
  ECAN_PARAM  
)
(MemberLink 
 STARTING_ATOM_STI_RENT
  ECAN_PARAM  
)
(MemberLink 
  STARTING_ATOM_LTI_RENT
  ECAN_PARAM  
)
(MemberLink 
  TARGET_STI_FUNDS
  ECAN_PARAM  
)

(MemberLink 
  STI_FUNDS_BUFFER
  ECAN_PARAM  
)

(MemberLink 
  LTI_FUNDS_BUFFER
  ECAN_PARAM  
)
(MemberLink 
  TARGET_LTI_FUNDS_BUFFER
  ECAN_PARAM  
)
(MemberLink 
  RENT_TOURNAMENT_SIZE
  ECAN_PARAM  
)

(StateLink
  AFB_SIZE
  (NumberNode "0.2")  
)
(StateLink 
  AFB_DECAY
  (NumberNode "0.05")
)
(StateLink 
  AFB_BOTTOM
  (NumberNode "50")
)
(StateLink 
  AF_SIZE
  (NumberNode "0.05")
)
(StateLink 
  MAX_AF_SIZE
  (NumberNode "500")
)
(StateLink 
  AF_RENT_FREQUENCY 
  (NumberNode "0.05")
)
(StateLink 
  FORGET_THRESHOLD
  (NumberNode "0.05")
)
(StateLink 
  MAX_LINKS
  (NumberNode "300")
)
(StateLink 
  HEBBIAN_MAX_ALLOCATION_PERCENTAGE
  (NumberNode "0.05")
)
(StateLink 
  LOCAL_FAR_LINK_RATIO
  (NumberNode "10")
)
(StateLink 
  MAX_SPREAD_PERCENTAGE
  (NumberNode "0.4")
)
(StateLink 
  SPREAD_HEBBIAN_ONLY
  (NumberNode "0")
)
(StateLink 
  DIFFUSION_TOURNAMENT_SIZE
  (NumberNode "5")
)
(StateLink 
 STARTING_ATOM_STI_RENT
  (NumberNode "1")
)
(StateLink 
  STARTING_ATOM_LTI_RENT
  (NumberNode "1")
)
(StateLink 
  TARGET_STI_FUNDS
  (NumberNode "10000")
)

(StateLink 
  STI_FUNDS_BUFFER
  (NumberNode "10000")
)

(StateLink 
  LTI_FUNDS_BUFFER
  (NumberNode "10000")
)
(StateLink 
  TARGET_LTI_FUNDS_BUFFER
  (NumberNode "10000")
)
(StateLink 
  RENT_TOURNAMENT_SIZE
  (NumberNode "5")
)

