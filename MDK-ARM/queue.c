#include "Queue.h"

void InitQueue(Queue *q)
{
    q->num_entries = 0;
    q->head = 0;
    q->tail = 0;
}

int QueueEmpty(Queue *q)
{
    return (q->num_entries == 0) ? 1 : 0;
}

int QueueFull(Queue *q)
{
    return (q->num_entries == QUEUE_MAX_SIZE) ? 1 : 0;
}

int Enqueue(Queue *q, Payload p)
{
    if(QueueFull(q))
    {
        return 0;
    }
    q->packets[q->tail] = p;
    q->num_entries++;
    q->tail = (q->tail + 1) % QUEUE_MAX_SIZE;
    return 1;
}

int Dequeue(Queue *q, Payload *payload)
{
    if(QueueEmpty(q))
    {
        return 0;
    }

    Payload p = q->packets[q->head];
    q->head = (q->head + 1) % QUEUE_MAX_SIZE;
    q->num_entries--;
    *payload = p;
    return 1;
}