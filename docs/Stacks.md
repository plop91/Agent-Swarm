# Stacks

A stack in the context of the agent swarm is a group of agents that complete a specific task.

## Stack Types

### Core Stack

The core stack is the stack that is responsible for the core functionality of the agent swarm. This includes the following:

```text
# data flows up the stack
0: output-node
2: 'hidden' nodes
1: input-node
```

### Agent Development Stack



### Correctness Stack

the correctness stack is the stack that is responsible for ensuring that the agent swarm is outputting correct information. This includes the following:

```text
0: correctness-node # the correctness node takes in the input thought and evaluates it for correctness.
1: verifier-node    # the verifier node takes in the input thought and the output of the correctness-node and evaluates the correctness-node's output to make sure the thoughts haven't changed.
```