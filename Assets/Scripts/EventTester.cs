//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public class EventTester : MonoBehaviour
//{
//    private TERGlove_driver ter_driver;

//    void Start()
//    {
//        //ter_driver = GetComponent<TERGlove_driver>();

//        //// Register an event handler for the DataReceived event
//        //TERGlove_driver.onMsgArrival += MsgReceived;
//    }

//    void MsgReceived(TER_packet data)
//    {
//        // Handle the received data here
//        Debug.Log("Received message: ");
//        //Debug.Log(data.quaternion[0]);
//        //Debug.Log(data.id);
//    }

//    void OnDisable()
//    {
//        // Unregister the event handler when the script is disabled
//        TERGlove_driver.onMsgArrival -= MsgReceived;
//    }
//}