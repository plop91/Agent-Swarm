
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from openai import OpenAI
import os
import time


class Agent(Node):

    def __init__(self):
        super().__init__("Agent")

        self.client = OpenAI()

        my_assistants = self.client.beta.assistants.list(
            order="desc",
            limit="20",
        )

        # which category is this agent
        self.declare_parameter('agent_category', '')
        agent_category = self.get_parameter(
            'agent_category').get_parameter_value().string_value

        self.category = agent_category

        # what is the name of the agent
        self.declare_parameter('agent_name', 'blank_agent')
        agent_name = self.get_parameter(
            'agent_name').get_parameter_value().string_value

        self.name = agent_name

        # which agent is this agent listening to
        self.declare_parameter('input_topic', 'blank_agent')
        input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        
        
        # which agent is this agent listening to
        self.declare_parameter('output_topic', 'blank_agent')
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value

        self.assistant = None

        for assistant in my_assistants.data:
            if assistant.name == agent_name:
                self.assistant = assistant
                break

        file_ids = []
        if agent_category == "":

            # get folders in the agent directory
            folders = os.listdir(os.path.join(os.getcwd(), "Agents"))

            if agent_name not in folders:
                raise Exception(
                    f"Agent {agent_name} not found in {os.path.join(os.getcwd(), 'Agents')}")

            # check that instructions.md exists
            if "instructions.md" not in os.listdir(os.path.join(os.getcwd(), "Agents", agent_name)):
                raise Exception(
                    f"Agent {agent_name} does not have an instructions.md file.")

            # read instructions.md
            self.instructions = open(os.path.join(
                os.getcwd(), "Agents", agent_name, "instructions.md"), "r").read()

            if self.instructions == "":
                raise Exception(
                    f"Agent {agent_name} does not have any instructions in the instructions.md file.")

            # check if files directory exists
            if "files" not in os.listdir(os.path.join(os.getcwd(), "Agents", agent_name)):
                raise Exception(
                    f"Agent {agent_name} does not have a files directory.")

            # get the files in the files directory
            files_in_dir = os.listdir(os.path.join(
                os.getcwd(), "Agents", agent_name, "files"))

            # get the files in the files accessible by on openai
            oai_files = self.client.files.list().data

            for file in files_in_dir:
                file_found = False
                for file_online in oai_files:
                    if file == file_online.filename:
                        file_found = True
                        file_ids.append(file_online.id)
                        break
                if not file_found:
                    file = self.client.files.create(
                        file=open(os.path.join(
                            os.getcwd(), "Agents", agent_name, "files", file), "rb"),
                        purpose='assistants'
                    )
                    file_ids.append(file.id)

        else:

            # get folders in the agent directory
            folders = os.listdir(os.path.join(os.getcwd(), "Agents", agent_category))

            if agent_name not in folders:
                raise Exception(
                    f"Agent {agent_name} not found in {os.path.join(os.getcwd(), 'Agents', agent_category)}")

            # check that instructions.md exists
            if "instructions.md" not in os.listdir(os.path.join(os.getcwd(), "Agents", agent_category, agent_name)):
                raise Exception(
                    f"Agent {agent_name} does not have an instructions.md file.")

            # read instructions.md
            self.instructions = open(os.path.join(
                os.getcwd(), "Agents", agent_category, agent_name, "instructions.md"), "r").read()

            if self.instructions == "":
                raise Exception(
                    f"Agent {agent_name} does not have any instructions in the instructions.md file.")

            # check if files directory exists
            if "files" not in os.listdir(os.path.join(os.getcwd(), "Agents", agent_category, agent_name)):
                raise Exception(
                    f"Agent {agent_name} does not have a files directory.")

            # get the files in the files directory
            files_in_dir = os.listdir(os.path.join(
                os.getcwd(), "Agents", agent_category, agent_name, "files"))

            # get the files in the files accessible by on openai
            oai_files = self.client.files.list().data

            for file in files_in_dir:
                file_found = False
                for file_online in oai_files:
                    if file == file_online.filename:
                        file_found = True
                        file_ids.append(file_online.id)
                        break
                if not file_found:
                    file = self.client.files.create(
                        file=open(os.path.join(
                            os.getcwd(), "Agents", agent_name, "files", file), "rb"),
                        purpose='assistants'
                    )
                    file_ids.append(file.id)
        

        # for file in files_in_dir:
        #     if file not in oai_files:
        #         raise Exception(
        #             f"Agent {agent_name} does not have access to file {file}.")

        # TEMP: Delete the active assistant no matter what
        if self.assistant is not None:
            self.client.beta.assistants.delete(self.assistant.id)
            self.assistant = None

        # if an assistant was not found with the name, create a new one
        if self.assistant is None:
            self.assistant = self.client.beta.assistants.create(
                name=agent_name,
                instructions=self.instructions,
                tools=[{'type': 'code_interpreter'}, {'type': 'retrieval'}],
                model="gpt-4-1106-preview",
                file_ids=file_ids
            )

        # create a thread for the agent
        # Note: a new for each instance of the agent
        self.thread = self.client.beta.threads.create()

        self.publisher_ = self.create_publisher(
            String, f'{output_topic}_thought', 10)

        self.get_logger().info(
            f"Agent {self.name} publishing on '{output_topic}_thought'")

        self.subscription = self.create_subscription(
            String,
            f'{input_topic}_thought',
            self.listener_callback,
            10)

        self.get_logger().info(
            f"Agent {self.name} listening to '{input_topic}_thought'")

        self.subscription

        self.get_logger().info(f"Agent {self.name} has been created.")

    def listener_callback(self, msg):

        self.get_logger().info(f"I heard: '{msg.data}'")

        # concatenate input data into a single string with spaces between each item
        # data = "".join([str(item) + " " for item in input_data])
        data = msg.data

        # add a message to the thread
        self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=data
        )

        # run the assistant
        run = self.client.beta.threads.runs.create(
            thread_id=self.thread.id,
            assistant_id=self.assistant.id,
            instructions=self.instructions
        )

        self.get_logger().info(f"Agent waiting for assistant to complete.")

        # wait for the run to complete
        run = self.client.beta.threads.runs.retrieve(
            thread_id=self.thread.id,
            run_id=run.id
        )
        while run.status != "completed":
            time.sleep(0.1)
            run = self.client.beta.threads.runs.retrieve(
                thread_id=self.thread.id,
                run_id=run.id
            )
        

        messages = self.client.beta.threads.messages.list(
            thread_id=self.thread.id
        )

        # test what kind of message it is and get the response
        if messages.data[0].content[0].type == "text":
            response = messages.data[0].content[0].text.value
        elif messages.data[0].content[0].type == "MessageContentImageFile":
            response = "Image file not supported yet."
            self.get_logger().info(f"Image file not supported yet.")
        else:
            raise Exception(
                f"Message type '{messages.data[0].content[0].type}' not supported.")

        self.get_logger().info(f"response:{response}")

        msg = String()
        msg.data = response

        self.publisher_.publish(msg)

        # append to agent.log
        with open(os.path.join(os.getcwd(), "Agents", self.name, "agent.log"), "a") as f:
            f.write(f"{time.time()},{self.name},{response}\n")


def main(args=None):
    rclpy.init(args=args)

    agent = Agent()
    
    rclpy.spin(agent)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
