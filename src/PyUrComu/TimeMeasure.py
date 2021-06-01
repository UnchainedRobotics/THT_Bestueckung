import time
def measureTime(func):
    def wrapper(*args, **kwargs):
        start_time = time.monotonic()
        ret = func(*args, **kwargs)
        elapsedTime=time.monotonic()-start_time
        Out=[elapsedTime,ret]
        return Out
    return wrapper

def getTimeStamp(func):
    def wrapper(*args, **kwargs):
        ret = func(*args, **kwargs)
        elapsedTime=time.monotonic()
        Out=[elapsedTime,ret]
        return Out
    return wrapper


