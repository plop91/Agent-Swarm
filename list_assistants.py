from openai import OpenAI

api_key = "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"

client = OpenAI(api_key=api_key)

assistants = client.beta.assistants.list().data

for assistant in assistants:
    print(f"\
Assistant ID: {assistant.id}\n\
Name:{assistant.name}\n\
instructions: {assistant.instructions.strip()}\n\
File IDs: {assistant.file_ids}\n\
")
    print(assistant)


assistant = assistants[0]

thread = client.beta.threads.create()

message = client.beta.threads.messages.create(
    thread_id=thread.id,
    role="user",
    content="I need to solve the equation `3x + 11 = 14`. Can you help me?"
)

print(message)

run = client.beta.threads.runs.create(
  thread_id=thread.id,
  assistant_id=assistant.id,
  instructions="Please address the user as Jane Doe. The user has a premium account."
)

print(run)

run = client.beta.threads.runs.retrieve(
  thread_id=thread.id,
  run_id=run.id
)

print(run)