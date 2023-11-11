"""
Agent.py
Author: Ian Sodersjerna
Date: 9/10/2023
This file contains a class that represents an agent.
"""
from openai import OpenAI
import time
import threading

print_lock = threading.Lock()


class Agent(threading.Thread):
    """
    This class represents an agent.
    """

    input_Agents = []
    input_agents_last_thought_id = {}

    output_Agents = []

    thoughts_mutex = threading.Lock()
    last_thought_id = "0"
    thoughts = {"0": ""}

    def __init__(self, client, name: str = "Agent"):
        """
        This is the constructor.
        """
        super().__init__()

        self.name = name
        self.continue_running = True

        # save a reference to the client
        self.client = client



        my_assistants = self.client.beta.assistants.list(
            order="desc",
            limit="20",
        )

        self.assistant = None

        for assistant in my_assistants.data:
            if assistant.name == "test agent creator":
                self.assistant = assistant
                break

        if self.assistant is None:
            self.assistant = client.beta.assistants.create(
                name="test agent creator",
                instructions="You write python files that are used as part of an AI swarm that is used to create a "
                             "new AI swarm.",
                tools=[{"type": "code_interpreter"}],
                model="gpt-4-1106-preview"
            )

        # create a thread for the agent
        # Note: a new for each instance of the agent
        self.thread = self.client.beta.threads.create()

    def add_input_agent(self, agent):
        """
        This method adds an input agent.
        :param agent: input agent
        """
        self.input_Agents.append(agent)
        self.input_agents_last_thought_id[agent] = "0"

    def run(self):
        """
        This method makes the agent active, and it will run until it is killed.
        """
        with print_lock:
            print(f"Agent:'{self.name}' is running.")

        while self.continue_running:

            if not self.is_input_data_ready():
                time.sleep(0.1)
                continue

            input_data = self.gather_input_data()

            self.think(input_data)

    def think(self, input_data: list):
        """
        This method makes the agent think.
        """
        with print_lock:
            print(f"Agent {self.name} is thinking.")
        # concatenate input data into a single string with spaces between each item
        data = "".join([str(item) + " " for item in input_data])

        with print_lock:
            print(f"Agent {self.name} is sending '{data}' to the assistant.")

        # add a message to the thread
        message = self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=data
        )

        with print_lock:
            print(f"Message 1:{message}")

        run = self.client.beta.threads.runs.create(
            thread_id=self.thread.id,
            assistant_id=self.assistant.id,
            instructions="Please help the user in creating and upgrading an agent"
        )

        with print_lock:
            print(f"run 1:{run}")

        run = self.client.beta.threads.runs.retrieve(
            thread_id=self.thread.id,
            run_id=run.id
        )

        with print_lock:
            print(f"run 2:{run}")

        messages = self.client.beta.threads.messages.list(
            thread_id=self.thread.id
        )

        with print_lock:
            print(f"Message 2:{messages}")

        for agent in self.output_Agents:
            agent.continue_running = False
        for agent in self.input_Agents:
            agent.continue_running = False
        self.continue_running = False

        self.last_thought_id = str(int(self.last_thought_id) + 1)

    def is_input_data_ready(self):
        """
        This method checks if all input agents have data ready.
        :return:
        """
        for agent in self.input_Agents:
            agent.thoughts_mutex.acquire()
            if agent.last_thought_id == self.input_agents_last_thought_id[agent]:
                agent.thoughts_mutex.release()
                return False
            else:
                print(
                    f"Agent {self.name} has data ready:\nid:{agent.last_thought_id}\ndata:{agent.get_thought(agent.last_thought_id)}")
                agent.thoughts_mutex.release()
        return True

    def gather_input_data(self):
        """
        This method gathers data from all input agents.
        """
        input_data = []
        for agent in self.input_Agents:
            agent.thoughts_mutex.acquire()
            input_data.append(agent.get_thought(agent.last_thought_id))
            self.input_agents_last_thought_id[agent] = agent.last_thought_id
            agent.thoughts_mutex.release()
        return input_data

    def get_thought(self, thought_id: str):
        """
        This method returns the agent's thought.
        :return: thought
        """
        return self.thoughts[thought_id]


if __name__ == '__main__':

    api_key = "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"
    c = OpenAI(api_key=api_key)

    input_agent = Agent(c, name="Input Agent")
    middle_agent = Agent(c, name="Middle Agent")
    # output_agent = Agent(c, name="Output Agent")

    # output_agent.add_input_agent(middle_agent)
    middle_agent.add_input_agent(input_agent)
    # middle_agent.output_Agents.append(output_agent)

    input_agent.start()
    middle_agent.start()
    # output_agent.start()

    time.sleep(1)

    input_agent.thoughts_mutex.acquire()
    input_agent.thoughts["1"] = "Hello"
    input_agent.last_thought_id = "1"
    input_agent.thoughts_mutex.release()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard Interrupt, stopping agents.")
        input_agent.continue_running = False
        middle_agent.continue_running = False
        # output_agent.continue_running = False

    input_agent.join()
    middle_agent.join()
    # output_agent.join()

    exit()
