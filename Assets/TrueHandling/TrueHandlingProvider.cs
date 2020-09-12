using Leap;
using Leap.Unity;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Gets hand data from Leap Motion
 * Also contains HandStateSource interface for other sources
 */

public interface HandStateSource
{
    HandState GetHandState(bool right);
    event System.EventHandler FixedUpdateDone;
}

public class TrueHandlingProvider : MonoBehaviour, HandStateSource
{
    //public bool useFixedUpdate = true;

    public bool keepHandsWhenNotSeen = true;

    Leap.Unity.LeapServiceProvider _provider;
    Leap.Controller _controller;

    HandState _left;
    HandState _right;

    void Start()
    {
        _provider = FindObjectOfType<Leap.Unity.LeapServiceProvider>();

        if (_provider == null)
        {
            throw new System.Exception("Can't find leap motion provider");
        }

        //_controller = _provider.GetLeapController();
        //_controller.FrameReady += OnFrameReady;
        
        //if (useFixedUpdate) {
            _provider.OnFixedFrame += OnFrameReady;
        //} else {
        //    _provider.OnUpdateFrame += OnFrameReady;
        //}

        _left = new HandState();
        _right = new HandState();
        _right.right = true;
    }

    //TODO: Smoothing? do it here if so
    public HandState GetHandState(bool right)
    {
        return right ? _right : _left;
    }

    public event System.EventHandler FixedUpdateDone;

    void OnFrameReady(Frame frame)
    {
        if (_provider == null)
        {
            return;
        }

        bool nextLeftHandActive = false;
        bool nextRightHandActive = false;

        foreach (Leap.Hand hand in frame.Hands)
        {
            if (hand.IsLeft)
            {
                nextLeftHandActive = true;
            } else {
                nextRightHandActive = true;
            }

            HandState relevantState = hand.IsLeft ? _left : _right;

            relevantState.UpdateFromLeapHand(hand);
        }

        _left.active = (keepHandsWhenNotSeen && _left.active) || nextLeftHandActive;
        _left.notLost = nextLeftHandActive;
        _right.active = (keepHandsWhenNotSeen && _right.active) || nextRightHandActive;
        _right.notLost = nextRightHandActive;

        FixedUpdateDone(this, null);
    }

    void Update()
    {

    }
}
