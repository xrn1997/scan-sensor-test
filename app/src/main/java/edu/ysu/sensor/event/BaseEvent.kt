package edu.ysu.sensor.event

/**
 * EventBus事件基类
 * @author xrn1997
 * @date 2021/6/3
 */
open class BaseEvent<T>(
    var msg: String,
    var data: T
)





