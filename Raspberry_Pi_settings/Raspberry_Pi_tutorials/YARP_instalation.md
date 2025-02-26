1. go to page with instalation instructions
2. use this command ``` sudo sh -c 'echo "deb http://www.icub.eu/debian `lsb_release -cs` contrib/science" > /etc/apt/sources.list.d/icub.list'```
3. probably will have problem with the version of distribution so change it in  **sudo nano /etc/apt/sources.list.d/icub.list** from bookworm to bullseye
4. download manually public key from __keyserver.ubuntu.com__  ```key: 57A5ACB6110576A6```
5. then use gpg to write the to the keyring location
    
    ```
    gpg --no-default-keyring --keyring /usr/share/keyrings/icub-archive-keyring.gpg --import /path/to/public-key-file.asc
    ```
6. then in the file ```/etc/apt/sources.list.d/icub.list``` change the line with repository accordingly

    ```
    deb [signed-by=/etc/apt/sources.list.d/icub.list] http://www.icub.eu/debian bullseye ....
    ```

7. if you have problem with architecture just add in the brackets amd64

    ```
    deb [signed-by=/etc/apt/sources.list.d/icub.list, amd64] http://www.icub.eu/debian bullseye ....
    ```

8. then try 

    ```
    sudo apt update
    ```