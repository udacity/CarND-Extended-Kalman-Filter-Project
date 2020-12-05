from google.cloud import pubsub_v1

# TODO(developer): Use an existing topic.
project_id = "your-project-id"
topic_id = "your-topic-id"

project_id = "tz-playground-bigdata"
topic_id = "lidar_radar"

publisher_options = pubsub_v1.types.PublisherOptions(enable_message_ordering=True)

publisher = pubsub_v1.PublisherClient(publisher_options=publisher_options)

topic_path = publisher.topic_path(project_id, topic_id)

file = open('data/sample-laser-radar-measurement-data-1.txt', 'r')
lines = file.readlines()

futures = []
for line in lines:
    future = publisher.publish(topic_path, data=line.encode("utf-8"), ordering_key="demo")
    futures.append(future)

for future in futures:
    future.result()

print(f"Published messages with ordering keys to {topic_path}.")