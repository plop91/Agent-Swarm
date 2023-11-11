# Agent Swarm Structure

Ian Sodersjerna
Created: 11/9/2023
Based on: https://www.youtube.com/watch?v=VWh2_OlFvSw (OpenAI Agent Swarm: Let's build an autonomous agent swarm Pt 1: AGENTS THAT BUILD AGENTS by David Shapiro)

## Introduction

The purpose of this project is to develop an ai architecture that can work autonomously to determine the "best" output 
possible. In this context the best output will be defined not only based on its accuracy but also on its ethical and 
moral decisions. The Agent will be technically proficient as it will assimilate information from external sources as well

## Data Flow: Bubble up

The data flow will be a bubble up architecture. This means that the data will flow from the bottom up. the data passed 
from agent to agent will be strings, at the bottom level there will be a Agent responcible for receiving information 
(described later in the documenttion as the prompting agent).

## Data Transfer

data transfer will be paramount in the agent swar, we want to archive all the thoughts the swarm has so we can analyze 
the behavior at a later time. the data  will need to be fast to access.

Initially the data will be stored in a sqlite database, this will allow for fast access and easy storage. Each Agent will
have its own table in the database, this will allow for easy access to the data. An example of data in the database is

```text
| thought_id:int | agent_id:int | datetime:datetime | thought:str | metadata:str | providing_agent_id:list(int) | upstream_agent_id:list(int) |
```

NOTE: an id facility will be necessary to provide each thought with a unique id.

## Swarm Architecture

The swarm will consist of several levels with the top levels having the most power and the bottom levels having the least.
The top level will be known as the top level council, they will be responsible for the health and operation of the swarm.
The next level will be the Chiefs of knowledge, each chief covers a domain of knowledge, these chiefs will be responsible
for the operation of the agents in their domain. The next level will be the agents, these agents will be responsible for
bubbling up relevant information. finally the bottom level agents will be where text prompts are generated, initially 
this will only be the prompting agent but more agents will be added.

### Top Level Council

Agents in the top level council will be responsible for the health and operation of the swarm. They will be responsible
for the creation of new agents, the termination of agents, the deployment of agents, the monitoring of agents, and the
security of the swarm. The only levels the top level council will not have control over is other top level councils, 
the chiefs of knowledge, and the prompting agent.

#### Agent: agent creator

The agent creator will be responsible for the creation of new agents. The agent creator will be responsible for the 
editing of the agents.

#### Agent: agent evaluator

The agent ensures that the agents are operating correctly. The agent evaluator will "sandbox" a new agent to ensure that
it is operating correctly. if the agent is not operating correctly the agent evaluator will terminate the agent and give
the reason to the agent creator.

#### Agent: agent deployer

The agent deployer will be responsible for the deployment of agents. The agent deployer will insert an agent into the 
swarm and make sure the swarm is communicating correctly after the insertion.

#### Agent: agent terminator

The agent terminator will be responsible for the termination of agents. The agent terminator is the only agent that can 
remove an agent from the swarm. The agent terminator will be responsible for the removal of agents but not evaluating
the agents.

#### Agent: agent monitor

The agent monitor will be responsible for the monitoring of agents. The agent monitor will ensure that the agents are
operating correctly and that the agents are not doing anything malicious.

#### Agent: role manager

The role manager is part of the security of the swarm. The role manager will be responsible for the assignment of roles
to Agents. The role manager will be responsible for the creation of new roles and the editing of roles.

#### Agent: permission manager

The permission manager is part of the security of the swarm. The permission manager will be responsible for the assignment
of permissions to roles. The permission manager will be responsible for the creation of new permissions and the editing
of permissions.

#### Agent: level manager

The level manager is part of the security of the swarm. The level manager will be responsible for the assignment of levels
to agents. The level manager will be responsible for the creation of new levels and the editing of levels.

#### Agent: security manager

The security manager is part of the security of the swarm. The security manager looks at the security of the swarm as a
whole. The security manager will be responsible for the creation of new security measures/policies and the editing of 
security measures.

#### Agent: flow manager

The flow manager is a monitoring agent. The flow manager will be responsible for the monitoring of the data flow, if the
monitoring agent detects a problem or an inefficiency it will report it to the agent evaluator.

#### Agent: data manager

The data manager is responsible for the storage of data. The data manager will be responsible for the creation of new
data storage facilities and the editing of data storage facilities.

#### Agent: data analyzer

The data analyzer is responsible for the analysis of data. the primary purpose of the data analyzer is to analyze the
truthfulness of the data.

### Chiefs of Knowledge

#### Agent: chief of sciences

#### Agent: chief of literature

#### Agent: chief of history

#### Agent: chief of ethics

#### Agent: chief of philosophy

#### Agent: chief of relationships

#### Agent: chief of politics

#### Agent: chief of human interaction

#### Agent: chief of information gathering

#### Agent: chief of subject-matter-experts

### 'Hidden' Agents

### Bottom Level Agents

#### Agent: prompting agent


## Agent Architecture

## OpenAI assistant notes

## Agent functions

## Permissions

## Roles

## Level hierarchy

## Starting location

The first agent should be the agent creator, this should be the only agent that is created entirely by a human. The other
agents should be created by the agent creator and use it as a template

1. write a python class that can interact with the OpenAI API and create 'assistants'
2. give the model initial instructions
3. select the model that will be used
4. evaluate available tools
5. create a basic function that can be used to create a new agent
6. determine how to pass the Agent text documents

## Notes

### 11/10/2023

github copilot cli can be used to generate code that interacts with a linux cli.