"""
This is the main file of the program.
"""
from openai import OpenAI
import os

api_key = "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"


class CreationAgent:
    """

    """

    def __init__(self):
        self.client = OpenAI(api_key=api_key)
        self.assistant = None

    def create(self, name: str, prompt: str):
        """

        :param name:
        :param prompt:
        """
        self.assistant = self.client.beta.assistants.create(
            name=name,
            instructions=prompt,
            tools=[{"type": "code_interpreter"}],
            model="gpt-4-1106-preview"
        )

        thread = self.client.beta.threads.create()

        message = self.client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content="I need you to make an agent for me."
        )

        print(message)

        run = self.client.beta.threads.runs.create(
            thread_id=thread.id,
            assistant_id=self.assistant.id,
            instructions="Please address the user as Jane Doe. The user has a premium account."
        )

        run = self.client.beta.threads.runs.retrieve(
            thread_id=thread.id,
            run_id=run.id
        )

        messages = self.client.beta.threads.messages.list(
            thread_id=thread.id
        )


def main():
    """
    This is the main function of the program.
    """
    ca = CreationAgent()
    ca.create("test agent creator", "you create agents for an AI Agent Swarm")


if __name__ == "__main__":
    main()
