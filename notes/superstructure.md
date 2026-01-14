```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
graph TD;

IDLE

subgraph BALL

IDLE --> |intakeReq| INTAKE

READY --> |intakeReq + notFull| INTAKE

READY --> |feedReq| FEED

READY --> |scoreReq| SCORE

FEED --> |noFeedReq / notEmpty| READY

SCORE --> |noFeedReq / notEmpty + notOurTurn| READY

INTAKE --> |continuous toggle + feedReq| FEED_FLOW

INTAKE --> |continuous toggle + scoreReq| SCORE_FLOW

INTAKE --> |full / noIntakingReq| READY

SCORE_FLOW --> |noScoreRep + notEmpty| READY

FEED_FLOW --> |noFeedRep + notEmpty| READY

FEED_FLOW --> |empty + noFeedReq| IDLE

SCORE_FLOW --> |empty + noScoreReq / notOurTurn| IDLE

FEED --> |empty + noScoreReq| IDLE

SCORE --> |empty + noScoreReq / notOurTurn| IDLE

IDLE --> |feedReq + continuous toggle| FEED_FLOW

IDLE --> |scoreReq + continuous toggle| SCORE_FLOW

subgraph Scoring / Feeding [SHOOT]

SCORE <--> |continuous toggle| SCORE_FLOW

FEED <--> |continuous toggle| FEED_FLOW
end

end
subgraph ANTI_JAM

IDLE --> SPIT
READY --> SPIT
SCORE --> SPIT
FEED --> SPIT
INTAKE --> SPIT
SCORE_FLOW --> SPIT
FEED_FLOW --> SPIT

SPIT --> IDLE

end