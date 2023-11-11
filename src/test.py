import openai
import os


class OpenAIAssistantCreator:
    """

    """
    def __init__(self, api_key):
        self.api_key = api_key
        self.client = openai.OpenAI(api_key=self.api_key)

    def create_assistant(self, agent_name, instructions, model, tools, files_folder=None):
        """

        :param agent_name:
        :param instructions:
        :param model:
        :param tools:
        :param files_folder:
        :return:
        """
        # Check for the 'files' subfolder and process its contents
        file_ids = []
        if files_folder and os.path.isdir(files_folder):
            for filename in os.listdir(files_folder):
                file_path = os.path.join(files_folder, filename)
                with open(file_path, 'rb') as file_data:
                    # Upload each file to OpenAI
                    file_object = self.client.File.create(file=file_data, purpose='fine-tune')
                    file_ids.append(file_object['id'])

        # Display agent name and instructions
        print(agent_name)
        print(instructions)

        # Display list of files if present
        if file_ids:
            print(f"Files: {', '.join(file_ids)}")

        # Create Assistant parameters
        create_params = {
            "name": agent_name,
            "instructions": instructions,
            "model": model,
            "tools": tools,
            "file_ids": file_ids  # Include if there are any files
        }

        # Create the Assistant using the OpenAI API
        assistant = self.client.Assistant.create(**create_params)
        print("Assistant ID:", assistant['id'])
        return assistant['id']


# Retrieve API key from environment variable
api_key = os.getenv('OPENAI_API_KEY')
if api_key is None:
    raise ValueError('The OPENAI_API_KEY environment variable is not set.')

# Create an instance of the OpenAIAssistantCreator class
assistant_creator = OpenAIAssistantCreator(api_key)

# Define the agent's name, instructions, model, tools, and optional files folder
agent_name = 'MyAgent'
instructions = 'Please help me answer user queries.'
model = 'gpt-4-1106-preview'
tools = [{'type': 'code_interpreter'}, {'type': 'retrieval'}]

# Path to the folder containing any files to upload
files_folder = 'path/to/files'

# Create a new Assistant
assistant_id = assistant_creator.create_assistant(agent_name, instructions, model, tools, files_folder)
