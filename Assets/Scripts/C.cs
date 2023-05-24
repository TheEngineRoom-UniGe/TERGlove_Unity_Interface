using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class C : MonoBehaviour
{
    private A a;
    // Start is called before the first frame update
    void Start()
    {
        this.a = this.gameObject.GetComponent<A>();
    }

    // Update is called once per frame
    void Update()
    {
        this.a.a++;
    }
}
