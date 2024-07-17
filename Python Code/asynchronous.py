import asyncio

'''
This takes a total of 4 seconds  to run. 
The 1st coroutine runs when result1 is awaited. It takes 2 seconds.

The 2nd coroutine runs when result2 is awaited. It also takes 2 seconds.

These are not running simultaneously.
'''

async def fetch_data(delay, id):
    print("Fetching data....", id)
    await asyncio.sleep(delay)
    print("Data fetched id: ", id)
    return {"data": "Some data", "id": id}

# async def main():
#     task1 = fetch_data(2, 1)
#     task2 = fetch_data(2, 2)

#     result1 = await task1
#     print("Received result", result1)
    
#     result2 = await task2
#     print("Received result", result2)
    

'''
This takes a total of 3 seconds to run. As soon as one task is idle or waiting on something, 
other tasks starts to run. if we used the anove model, it would take a total of 2 + 2 + 3 = 7 seconds
'''

# async def main():
#     task1 = asyncio.create_task(fetch_data(2, 1))
#     task2 = asyncio.create_task(fetch_data(2, 2))
#     task3 = asyncio.create_task(fetch_data(3, 3))

#     result1, result2, result3 = await task1, await task2, await task3
#     print(result1, result2, result3)

'''
An alternative to manually creating tasks is to use TaskGroup(). TaskGroup creates tasks and automatically executes them.

'''

# async def main():
#     tasks = []
#     async with asyncio.TaskGroup() as tg:
#         for i, sleep_time in enumerate([2, 1, 3], start=1):
#             task = tg.create_task(fetch_data(i, sleep_time))
#             tasks.append(task)

#     results = [task.result() for task in tasks]
#     for result in results:
#         print(f"Result received : {result}")

'''
Locks ensure that any resource that could potentially be shared between co routines is accessed
by one co routine at a time.
'''

lock = asyncio.Lock()
 
async def modify_shared_resource():
    global shared_resource 

    async with lock: # Context manager. This part of the code can be accessed by only one co routine at a time
        print(f"Resource before modification {shared_resource}")
        shared_resource += 1
        await asyncio.sleep(1)
        print(f"Resource after modification {shared_resource}")

async def main():
    await asyncio.gather(*(modify_shared_resource() for _ in range(5)))


asyncio.run(main())