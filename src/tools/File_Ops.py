"""
This module contains the FileOps class, which handles file operations for OpenAI agents.
"""
from openai import OpenAI


class FileOps:
    """
    This class handles file operations for OpenAI agents.
    """

    def __init__(self, client: OpenAI):
        """
        This is the constructor.
        """
        self.client = client

    def list_files(self):
        """
        This function lists the files on the OpenAI server.
        :return: list of files
        """
        try:
            return self.client.files.list().data
        except Exception as e:
            print(e)

    def upload_file(self, file_path: str, purpose: str = "assistants"):
        """
        This function uploads a file to the OpenAI server.
        :param file_path: path to the file to upload
        :param purpose: the purpose of the file
        :return: None
        """
        try:
            if purpose == "assistants":
                self.client.files.create(
                    file=open(file_path, "rb"),
                    purpose="assistants"
                )
            elif purpose == "fine-tune":
                self.client.files.create(
                    file=open(file_path, "rb"),
                    purpose="fine-tune"
                )
            else:
                raise ValueError("Invalid purpose.")
        except Exception as e:
            print(e)

    def delete_file(self, file_id: str):
        """
        This function deletes a file from the OpenAI server.
        :param file_id: file ID
        :return: None
        """
        try:
            self.client.files.delete(file_id)
        except Exception as e:
            print(e)

    def download_file(self, file_id: str):
        """
        This function downloads a file and its contents from the OpenAI server.
        :param file_id:
        :return:
        """
        try:
            # file = self.client.files.retrieve(file_id)
            # contents = self.client.files.retrieve_content(file_id)
            # return file, contents

            return self.client.files.retrieve_content(file_id)

        except Exception as e:
            print(e)


if __name__ == '__main__':
    import os
    # if on Windows, use dotenv
    if os.name == "nt":
        import dotenv
        dotenv.load_dotenv()

    api_key = os.getenv('OPENAI_API_KEY')
    OpenAI_client = OpenAI(api_key=api_key)
    file_ops = FileOps(client=OpenAI_client)

    files = file_ops.list_files()
    file_ids = []
    for file in files:
        print(file.id)
        print(file.filename)
        print(f"{file.bytes/1000}KB")
        print(file_ops.download_file(file.id))
        print("=========================================")


