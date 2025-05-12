#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
import openai
from hri_hand_control.srv import HandCommand

def callback_gpt_input(msg):
    """
    Callback that fires whenever a message appears on /gpt_input.
    It sends the text prompt to ChatGPT, checks for hand commands,
    and publishes the response to /gpt_output and hand command to /hand_input.
    """
    user_input = msg.data
    rospy.loginfo("ROSGPT: Received input: %s", user_input)

    # Get the ChatGPT response, using the synonym context
    response_text = query_openai_chatgpt_with_synonyms(user_input)

    # Publish the response to /gpt_output
    pub_gpt_output.publish(response_text)
    rospy.loginfo("ROSGPT: Published response: %s", response_text)

    # Check if the response contains a command for the hand
    hand_command = parse_hand_command(response_text)
    if hand_command:
        rospy.loginfo(f"ROSGPT: Parsed hand command: {hand_command}")

        try:
            rospy.wait_for_service('/hand_command', timeout=5)
            hand_command_service = rospy.ServiceProxy('/hand_command', HandCommand)
            resp = hand_command_service(hand_command)
            rospy.loginfo("ROSGPT: Hand command service response: success=%s, message=%s", resp.success, resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("ROSGPT: Service call failed: %s", str(e))
        except rospy.ROSException:
            rospy.logerr("ROSGPT: Service /hand_command not available.")
    else:
        rospy.loginfo("ROSGPT: No valid hand command found in response.")



def query_openai_chatgpt_with_synonyms(user_input):
    """
    Send the user's input to the OpenAI API, initializing with the synonym dictionary context.
    """
    synonym_dict = """
    You are a control interface for a robotic hand. Here is a dictionary of internal commands and their synonyms for you to process:
    "full_close": "close hand", "make a fist", "clench", "grip", "grab", "full close", "full_close",
    "full_open": "open hand", "release", "let go", "unclench", "spread fingers", "full open", "full_open", "open", "stop",
    "thumbs_up": "thumbs up", "give thumbs up", "raise thumb", "thumbs up gesture", "thumbs_up", "good job", "well done",
    "tri_close": "tri grip", "tri close", "tri grip close", "tri close grip", "tri_close", "three fingers close", "three fingers grip",
    "tri_open": "tri_open", "three fingers open",
    "pinch": "pinch", "ok sign", "okay sign", "pinch gesture", "pinch close",
    "pinch_open": "unpinch", "release pinch", "open pinch", "pinch open",
    "point_gesture": "pointing gesture", "point", "point_gesture",
    "peace_gesture": "peace sign", "two fingers up", "v sign", "peace gesture", "peace",
    """

    prompt = f"""
    {synonym_dict}
    Please interpret the following user command based on the synonym dictionary:
    User Command: "{user_input}"
    """

    # Send the constructed prompt to GPT-3.5 API
    try:
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "system", "content": synonym_dict},
                      {"role": "user", "content": user_input}],
            max_tokens=150,
            temperature=0.7
        )
        return response['choices'][0]['message']['content'].strip()
    except Exception as e:
        rospy.logerr("Error querying OpenAI API: %s", str(e))
        return "Error: Could not retrieve response from OpenAI."

def parse_hand_command(text):
    text = text.lower()

    commands = {
        "full_close": ["close hand", "make a fist", "clench", "grip", "grab", "full close", "full_close"],
        "full_open": ["open hand", "release", "let go", "unclench", "spread fingers", "full open", "full_open", "open", "stop"],
        "thumbs_up": ["thumbs up", "give thumbs up", "raise thumb", "thumbs up gesture", "thumbs_up", "good job", "well done"],
        "tri_close": ["tri grip", "tri close", "tri grip close", "tri close grip", "tri_close", "three fingers close", "three fingers grip"],
        "tri_open": ["tri_open", "three fingers open"],
        "pinch": ["pinch", "ok sign", "okay sign", "pinch gesture", "pinch close"],
        "pinch_open": ["unpinch", "release pinch", "open pinch", "pinch open"],
        "point_gesture": ["pointing gesture", "point", "point_gesture"],
        "peace_gesture": ["peace sign", "two fingers up", "v sign", "peace gesture", "peace"],
        
    }

    for cmd, keywords in commands.items():
        for phrase in keywords:
            if phrase in text:
                return cmd

    return None



if __name__ == "__main__":
    # 1. Initialize the ROS node
    rospy.init_node("rosgpt_node")

    # 2. Retrieve the OpenAI API Key
    openai_api_key = os.environ.get('OPENAI_API_KEY')
    if not openai_api_key:
        rospy.logerr("OPENAI_API_KEY not found in the environment.")
        rospy.signal_shutdown("No OpenAI API Key provided.")
        exit(1)
    else:
        openai.api_key = openai_api_key

    # 3. Create a publisher for /gpt_output
    pub_gpt_output = rospy.Publisher("/gpt_output", String, queue_size=10)

    # 4. Create a publisher for hand commands (to send to the /hand_input topic)
    pub_hand_command = rospy.Publisher("/hand_input", String, queue_size=10)

    # 5. Subscribe to /gpt_input
    rospy.Subscriber("/gpt_input", String, callback_gpt_input)

    # 6. Keep the node running
    rospy.loginfo("ROSGPT: Node started. Awaiting input on /gpt_input.")
    rospy.spin()
    
    
    
