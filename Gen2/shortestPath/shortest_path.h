#include <string.h>
void SimplifyPath(char path[], char choice);
int DeleteElement(char *arr, int n, int pos)
{
    for (int i = pos; i < n - 1; i++)
        arr[i] = arr[i + 1];
    arr[n - 1] = '\0';
    return n - 1;
}

int InsertElement(char *arr, int n, int pos, int val)
{
    for (int i = n; i >= pos; i--)
        arr[i] = arr[i - 1];
    arr[pos] = val;
    return n + 1;
}
char RightHand(char a[])
{

    if (a[0] == 'R' && a[1] == 'B' && a[2] == 'L')
    {
        return 'B';
    }
    else if (a[0] == 'R' && a[1] == 'B' && a[2] == 'S')
    {
        return 'L';
    }
    else if (a[0] == 'R' && a[1] == 'B' && a[2] == 'R')
    {
        return 'S';
    }
    else if (a[0] == 'S' && a[1] == 'B' && a[2] == 'R')
    {
        return 'L';
    }
    else if (a[0] == 'S' && a[1] == 'B' && a[2] == 'S')
    {
        return 'B';
    }
    else if (a[0] == 'L' && a[1] == 'B' && a[2] == 'R')
    {
        return 'B';
    }
    else
    {
        return 0;
    }
}

char LeftHand(char a[])
{

    if (a[0] == 'L' && a[1] == 'B' && a[2] == 'R')
    {
        return 'B';
    }
    else if (a[0] == 'L' && a[1] == 'B' && a[2] == 'S')
    {
        return 'R';
    }
    else if (a[0] == 'L' && a[1] == 'B' && a[2] == 'L')
    {
        return 'S';
    }
    else if (a[0] == 'S' && a[1] == 'B' && a[2] == 'L')
    {
        return 'R';
    }
    else if (a[0] == 'S' && a[1] == 'B' && a[2] == 'S')
    {
        return 'B';
    }
    else if (a[0] == 'R' && a[1] == 'B' && a[2] == 'L')
    {
        return 'B';
    }
    else
    {
        return '\0';
    }
}
void ReducePath(char *path, int n, int index, char choice)
{
    //n = strlen(path);
    if (index >= 0 && index <= n)
    {
        char splice[] = {path[index], path[index + 1], path[index + 2]}, ans;

        if (choice == 'R')
            ans = RightHand(splice);
        else if (choice == 'L')
            ans = LeftHand(splice);

        if (ans != '\0')
        {
            for (int c = 0; c < 3; c++)
                n = DeleteElement(path, n, index);
            n = InsertElement(path, n, index, ans);
        }
    }
}

void SimplifyPath(char path[], char choice)
{

    //SSLLSRRRBRRBRRRBSRRLLR
    //SSLLSRRSRBRRRBSRRLLR
    //SSLLSRRSSRRBSRRLLR
    //SSLLSRRSSRLRRLLR
    int n = strlen(path);
    for (int i = 0; i < n; i++)
    {
        ReducePath(path, n, i, choice);
    }

    if (strlen(path) != n)
        SimplifyPath(path, choice);
}

void FilterPath(char *path)
{
    int n = strlen(path);

    for (int i = 0; i < n; i++)
    {
        if (path[i] == 'B' && path[i + 1] == 'B')
        {
            n = DeleteElement(path, n, i + 1);
            i--;
        }
    }
}