from openai import OpenAI
import os


class CreationAgent:
    """

    """

    def __init__(self, agents_path: str = 'agents'):
        api_key = os.getenv('OPENAI_API_KEY')
        if api_key is None:
            raise ValueError('The OPENAI_API_KEY environment variable is not set.')
        self.client = OpenAI(api_key=api_key)

        if not os.path.exists(agents_path) or not os.path.isdir(agents_path):
            os.mkdir(agents_path)

    def create(self, name: str, prompt: str):
        """
        Create an agent using the OpenAI API.

        :param name: The name of the agent.
        :param prompt: The prompt to use when creating the agent.
        """
