using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class B : MonoBehaviour
{
    private A a;
    // Start is called before the first frame update
    void Start()
    {
        this.a = this.gameObject.GetComponent<A>();
        Debug.Log(this.a);
    }

    // Update is called once per frame
    void Update()
    {
        this.a.a--;
        Debug.Log(this.a.a);
    }
}
