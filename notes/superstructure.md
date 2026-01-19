```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
graph TD;

IDLE

subgraph BALL

IDLE <--> |intakeReq + intakeEmpty| INTAKE

READY <--> |intakeReq + empty| INTAKE

READY --> |feedReq| SPIN_UP_FEED

READY --> |scoreReq| SPIN_UP_SCORE

SPIN_UP_FEED --> |atVelocity| FEED

SPIN_UP_SCORE --> |atVelocity| SCORE


FEED --> |flowReq| FEED_FLOW

SCORE --> |flowReq| SCORE_FLOW


FEED --> |empty| IDLE

SCORE --> |empty| IDLE

FEED <--> |position on field + no scoring when not our turn| SCORE

FEED_FLOW --> |empty| IDLE

SCORE_FLOW --> |empty| IDLE

end

subgraph ANTI_JAM

SPIT

end
