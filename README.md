# Website Cigritous

This website is a platform that is useful for monitoring and controlling drones in performing their tasks. This website consists of 3 pages, namely the dashboard page, about page, and control page. This website is connected to MQTT as communication for ESP and PostgreSQL for DBMS.

You can see the results of the finished website by clicking the following link: [Cigritous.tech](https://www.cigritous.tech/)

---

# Installation In Local

In the installation section, because this web is built on Node JS. So you must install Node JS first to run this website on your local computer And if you want to run this web normally for your personal gain, you must have MQTT for communication to ESP and PostgreSQL for Database. then, appropriate in this code like enviroment variabel and etc.

Install Node JS: [Node JS](https://nodejs.org/en/download)

## For Front End

In the project directory, you can run:

### `npm install`

Run this comment when node_modulous not show in your project directory.\
If error, please install node JS.

### `npm start`

Runs the app in the development mode.\
Open [http://localhost:3000](http://localhost:3000) to view it in your browser.

The page will reload when you make changes.\
You may also see any lint errors in the console.

### `npm test`

Launches the test runner in the interactive watch mode.\

### `npm run build`

Builds the app for production to the `build` folder.\
It correctly bundles React in production mode and optimizes the build for the best performance.

The build is minified and the filenames include the hashes.\
Your app is ready to be deployed!

## For Back End

In the project directory, you can run:

### `npm install`

Run this comment when node_modulous not show in your project directory.\
If error, please install node JS.

### `npm start`

Runs the app in the development mode.\
Open [http://localhost:5000](http://localhost:5000) to view it in your browser.

The page will reload when you make changes.\
You may also see any lint errors in the console.
And you can Add api for doing get or post data from database.
example : `http://localhost:5000/updatecentral` for get data central Node.

---

## Dashboard Page

Useful for determining many nodes, monitoring the position of the drone in real time, monitoring various available parameters such as temperature, humidity, moisture, and others. On this page there is also a record graph that is useful for viewing the history of the state of plant nodes.

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091361691545129101/image.png)

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091361618899775649/image.png)

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091348467907051570/image.png)

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091349327412203560/image.png)

## ![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091355107066646649/image.png)

---

## About page

This page contains some documentation of the drone at work and the background of the cigritous project.

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091340569759981648/image.png)

---

## Controls page

On this page the user can perform an activity related to the drone such as, take off, landing, and activate the crow detection mode. On this page users can also see the status of the drone such as drone status, battery status, drone location, drone speed, and others.

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091361881085722654/image.png)

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091361984332705863/image.png))

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091344335678750821/image.png)

![Gambar Dashboard](https://cdn.discordapp.com/attachments/1048974551440179331/1091344447054282843/image.png)

---
