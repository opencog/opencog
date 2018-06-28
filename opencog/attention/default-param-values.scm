
(use-modules (opencog))

(define AF_RENT_FREQUENCY  (ConceptNode "AF_RENT_FREQUENCY"))
(define MAX_AF_SIZE (ConceptNode "MAX_AF_SIZE"))
(define MIN_AF_SIZE (ConceptNode "MIN_AF_SIZE"))
(define AF_SIZE (ConceptNode "AF_SIZE"))
(define AFB_BOTTOM  (ConceptNode "AFB_BOTTOM"))
(define AFB_DECAY (ConceptNode "AFB_DECAY"))
(define ECAN_PARAM (ConceptNode "ECAN_PARAMS"))
(define FORGET_THRESHOLD (ConceptNode "FORGET_THRESHOLD") ) 
(define MAX_LINKS (ConceptNode "MAX_LINKS")   ) 
(define HEBBIAN_MAX_ALLOCATION_PERCENTAGE (ConceptNode "HEBBIAN_MAX_ALLOCATION_PERCENTAGE") )  
(define LOCAL_FAR_LINK_RATIO (ConceptNode "LOCAL_FAR_LINK_RATIO") )
(define MAX_SPREAD_PERCENTAGE  (ConceptNode "MAX_SPREAD_PERCENTAGE"))
(define SPREAD_HEBBIAN_ONLY (ConceptNode "SPREAD_HEBBIAN_ONLY"))
(define DIFFUSION_TOURNAMENT_SIZE(ConceptNode "DIFFUSION_TOURNAMENT_SIZE"))
(define STARTING_ATOM_STI_RENT (ConceptNode "STARTING_ATOM_STI_RENT"))
(define STARTING_ATOM_LTI_RENT (ConceptNode "STARTING_ATOM_LTI_RENT"))
(define TARGET_STI_FUNDS (ConceptNode "TARGET_STI_FUNDS"))
(define TARGET_LTI_FUNDS (ConceptNode "TARGET_LTI_FUNDS"))
(define STI_FUNDS_BUFFER (ConceptNode "STI_FUNDS_BUFFER"))
(define LTI_FUNDS_BUFFER (ConceptNode "LTI_FUNDS_BUFFER"))
(define TARGET_LTI_FUNDS_BUFFER (ConceptNode "TARGET_LTI_FUNDS_BUFFER"))
(define RENT_TOURNAMENT_SIZE (ConceptNode "RENT_TOURNAMENT_SIZE"))
(define SPREADING_FILTER (ConceptNode "SPREADING_FILTER"))

(MemberLink 
  ECAN_PARAM  
  AF_SIZE
)
(MemberLink
  ECAN_PARAM
  MAX_AF_SIZE
)
(MemberLink 
  ECAN_PARAM
  MIN_AF_SIZE  
)
(MemberLink 
  ECAN_PARAM
  AFB_DECAY
)
(MemberLink
  ECAN_PARAM
  AFB_BOTTOM
)
(MemberLink
  ECAN_PARAM
  MAX_AF_SIZE
)
(MemberLink 
  ECAN_PARAM
  AF_RENT_FREQUENCY  
)
(MemberLink 
  ECAN_PARAM
  FORGET_THRESHOLD
)
(MemberLink 
  ECAN_PARAM
  MAX_LINKS
)
(MemberLink 
  ECAN_PARAM
  HEBBIAN_MAX_ALLOCATION_PERCENTAGE
)
(MemberLink 
  ECAN_PARAM
  LOCAL_FAR_LINK_RATIO
)
(MemberLink 
  ECAN_PARAM
  MAX_SPREAD_PERCENTAGE
)
(MemberLink
  ECAN_PARAM
  SPREADING_FILTER
)
(MemberLink 
  ECAN_PARAM
  SPREAD_HEBBIAN_ONLY
)
(MemberLink 
  ECAN_PARAM
  DIFFUSION_TOURNAMENT_SIZE
)
(MemberLink 
  ECAN_PARAM
  STARTING_ATOM_STI_RENT
)
(MemberLink 
  ECAN_PARAM
  STARTING_ATOM_LTI_RENT
)
(MemberLink 
  ECAN_PARAM
  TARGET_STI_FUNDS
)
(MemberLink
  ECAN_PARAM
  TARGET_LTI_FUNDS
)
(MemberLink 
  ECAN_PARAM
  STI_FUNDS_BUFFER
)
(MemberLink 
  ECAN_PARAM
  LTI_FUNDS_BUFFER
)
(MemberLink 
  ECAN_PARAM
  TARGET_LTI_FUNDS_BUFFER
)
(MemberLink 
  ECAN_PARAM
  RENT_TOURNAMENT_SIZE
)
(StateLink
  AF_SIZE
  (NumberNode "0.2")
)
(StateLink
  MIN_AF_SIZE
  (NumberNode "500")
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
  MAX_AF_SIZE
  (NumberNode "1000")
)
(StateLink 
  AF_RENT_FREQUENCY 
  (NumberNode "5")
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
  SPREADING_FILTER
  (MemberLink     ; TODO: MemberLink should have 2 arguments
    (TypeNode "MemberLink")
  )
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
  TARGET_LTI_FUNDS
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

