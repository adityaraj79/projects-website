<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Inventrom Private Limited | Bolt Iot</title>
    
    <link href="https://fonts.googleapis.com/css2?family=Baloo+Bhai+2&family=Bree+Serif&display=swap" rel="stylesheet">


    <style>

        *{
    padding: 0px;
    margin: 0px;
}

#navbar{
    display: flex;
    align-items: center;
    position: relative;
    top: 0px;
}

#logo{
    margin: 3px 24px;
}

#logo img{
    margin: 3px 190px;
    height: 42px;

}

#navbar ul{
    display: flex;
}
#navbar::before{
    content: "";
    background-color: white;
    position: absolute;
    height: 100%;
    width: 100%;
    z-index: -2;
    opacity: 0.75;
    top: 0px;
    left: 0px;
}

#navbar ul li{
    list-style: none;
    font-size: 1.1rem;
}
#navbar ul li a{
    color: grey;
    font-family: 'Baloo Bhai 2', cursive;
    display: block;
    padding: 3px 15px;
    border-radius: 20px;
    text-decoration: none;
}
.laila{
    margin-left: 440px;
}
#navbar ul li a:hover{
    color: black;
}


#home{
    display: flex;
    flex-direction: column;
    padding: 3px 200px;
    justify-content: center;
    align-items: center;
    height: 597px;
}
#home::before{
    content: "";
    background:url('../background-layer.jpg');
    position: absolute;
    margin-top: 58px;
    top: 0px;
    left: 0px;
    width: 100%;
    height: 683px;
    z-index: -2;
    opacity: 0.85;
}

#home h1{
    color: white;
    text-align: center;
    font-family: 'Bree Serif', serif;
    padding: 23px;
}
#home p{
    color: white;
    text-align: center;
    font-size: 1.1rem;
    font-family: 'Bree Serif', serif;
    text-decoration: none;
}
#home p:hover{
    color: black;
}

#home .h-primary{
    font-size: 40px;
    font-weight: bold;
    padding: 39px;
    font-family: 'Bree Serif', serif;
}


#services{
    margin: 11px;
    display: flex;
}
#services-id{
    padding: 19px;
    margin: 80px 80px 0px 80px;
}

.services-primary{
    color: rgb(34, 90, 23);
    font-size: 23px;
    padding: 30px;
    height: 24px;
    font-family: 'Bree Serif', serif;
    text-align: center;
}

#services .box{
    padding: 34px;
    margin: 3px 35px;
    width: 100%;
}
#services .box img{
    height: 285px;
    width: 85%;
    margin: auto;
    display: block;
}

.h-secondary{
    font-size: 20px;
    padding: 12px;
    font-family: 'Bree Serif', serif;
    text-align: center;
}

#services .box p{
    font-family: 'Bree Serif', serif;
    font-size: 16px;
    text-align: center;
}

.taga{
    text-align: center;
    font-size: 16px;
    justify-content: center;
    color: black;
}


#grey_background{
    background-color: rgb(240, 240, 240);
    padding-bottom: 175px;
}

#awards .title{
    color: rgb(34, 90, 23);
    text-align: center;
    font-size: 23px;
    font-family: 'Bree Serif', serif;
    font-weight: bold;
    padding-top: 35px;
    padding-bottom: 100px;
}
#awards .sameline{
    display: flex;
    justify-content: space-evenly;
    align-items: center;
}
#awards .award img{
    height: 200px;
    width: 280px;
}
#awards .award{
    margin: -185px;
}

#awards h2{
    font-size: 13px;
    font-weight: bold;
    text-align: center;
}

#awards p{
    font-size: 11px;
    color: grey;
    text-align: center;
}
.contacts{
    color: rgb(34, 90, 23);
    text-align: center;
    font-size: 23px;
    font-family: 'Bree Serif', serif;
    font-weight: bold;
    padding-top: 15px;
    padding-bottom: 40px;
}

#contact{
    font-size: 13px;
    background-color: rgb(240, 240, 240);
    padding-bottom: 155px;
}
.form_fillup{
    width: 450px;
    float: left;
    padding-left: 285px;
}

.heading{
    color: black;
    font-size: 17px;
    font-weight: bold;
    margin-bottom: 30px;
}
.location{
    margin-left: 785px;
    color: rgb(99, 99, 99);
    margin-bottom: 30px;
}
.location_city{
    color: rgb(99, 99, 99);
    font-size: 14px;
}
.address{
    margin: 40px 0px;
    font-size: 14px;
}

.location img{
    height: 20px;
    width: 20px;
    margin-right: 10px;
}
.details{
    font-size: 14px;
    line-height: 25px;
}
.details p{
    text-align: center;
}

.footer{
    font-size: 14px;
    color: white;
    background-color: rgb(14, 94, 14);
    align-items: center;
    padding: 28px;
}
.copyright{
    margin-left: 200px;
}
.footer img{
    height: 20px;
    width: 20px;
    margin-right: 20px;
}

.up{
    display: inline;
    float: right;
}
.up img{
    margin-right: 230px;
}
.social img{
    margin-left: 680px;
}

    </style>

    
</head>
<body id="top">
    <nav id="navbar">
        <div id="logo">
            <a href="#navbar">
            <img src="logo.png" alt="Inventrom">
            </a>
        </div>
        <ul>
            <li class="item laila"><a href="#about">About</a></li>
            <li class="item"><a href="#awards">Awards</a></li>
            <li class="item"><a href="#contact">Contact</a></li>
        </ul>
    </nav>


    <section id="home">
        <h1 class="h-primary">Bolt is an Internet Of Things Platform for enterprise ready for Product and Solutions</h1>
        <a href="https://boltiot.com" target="_blank">
        <p>CLICK TO KNOW MORE</p>
        </a>
    </section>


    <section class="services-container" id="services-id">
    <div id="about">
        <h1 class="services-primary">ABOUT US</h1>
        <p class="taga">We are an award-winning IoT team that has extensive experience in enterprise IoT projects. Before working on the Bolt platform, we have
            <br>
        been strategic IoT consultants to various Fortune 500 companies.</p>
        <div id="services">
            <div class="box">
                <img src="about_us_vision.png" alt="">
                <h2 class="h-secondary center">Our Vision</h2>
                <p class="center">Build technology platforms for better communication.</p>
            </div>
            <div class="box">
                <img src="abous_us_mission.png" alt="">
                <h2 class="h-secondary center">Our Mission</h2>
                <p class="center">To build a Platform to make IoT development quick, easy and low cost
                    for developers. This will help in better connectivity
                    between humans and machines helping us all in getting the most
                    out of the machines. </p>
            </div>
        </div>
    </div>
    </section>
    

    <section id="grey_background">
        <div id="awards">
            <div class="title">AWARDS</div>
            <br>
            <div class="sameline">
                <div class="award">
                    <img src="award1.png" alt="">
                    <h2>SHOWCASE</h2>
                    
                    <p>_ _ _ _ _ _ _ _ _ _ _ _</p>
                    <p>INDO-US STARTUP KONNECT PROGRAM</p>
                    <p>SILICON VALLEY</p>
                </div>
                <div class="award">
                    <img src="award2.png" alt="">
                    <h2>WINNER</h2>
                    
                    <p>_ _ _ _ _ _ _ _ _ _ _ _</p>
                    <p>DST-LOCKHEED MARTIN INDIA INNOVATION</p>
                    <p>GROWTH PROGRAM</p>
                </div>
                <div class="award">
                    <img src="award3.png" alt="">
                    <h2>WINNER</h2>
                    
                    <p>_ _ _ _ _ _ _ _ _ _ _ _</p>
                    <p>POWER OF IDEAS</p>
                </div>
                <div class="award">
                    <img src="award4.png" alt="">
                    <h2>WINNER</h2>
                    
                    <p>_ _ _ _ _ _ _ _ _ _ _ _</p>
                    <p>CHIEF MINISTER AWARD FOR THE BEST START-UP IN</p>
                    <p>GOA</p>
                </div>
            </div>
        </div>

    </section>


    <div id="contact">
        <div class="contacts">GET IN TOUCH</div>
        <div class="form_fill">
            <div class="form_fillup">
                <!--[if lte IE 8]>
                <script charset="utf-8" type="text/javascript" src="//js.hsforms.net/forms/v2-legacy.js"></script>
                <![endif]-->

                <script charset="utf-8" type="text/javascript" src="//js.hsforms.net/forms/v2.js"></script>

                <script>

                hbspt.forms.create({

                portalId: "4801849",

                formId: "b021b49f-c504-4b1d-ad2b-1bb9e587287c"

                });

                </script>
            </div>


            <div class="location">
                <div class="heading">Our Locations</div>
                <div class="location_city">Mumbai</div>
                <br>
                <p class="address">
                    Inventrom Private Limited, Unit no: B 32-A, B-wing, 2nd Floor, Raj Industrial
                    <br>
                    Complex, Military Road, Marol, Andheri East, Mumbai, Maharashtra - 400059
                </p>
                <div class="location_city">Goa</div>
                <br>
                <p class="address">
                    Inventrom Private Limited, 10, Gasper Apartment, Behind Suzuki
                    <br>
                    Showroom, Panjim, Goa - 403001, Near People's High School
                </p>

            
                <div class="heading">Contact Us</div>
                <span class="details">
                    <img src="placeholder.png" alt="">
                    <span>
                        79, 11th Cross Road, Binnamangala Indiranagar, Bengaluru, Karnataka,560038, India
                    </span>
                </span>
                <br>
                <span class="details">
                    <img src="phone-call.png" alt="">
                    <span>
                        (+91) 8881197198
                    </span>
                </span>
                <br>
                <br>
                <span class="details">
                    <img src="email.png" alt="">
                    <span>
                        support@boltiot.com
                    </span>
                </span>
            </div>
        </div>
    </div>

    <div class="footer">
        <div class="copyright">@ 2021 Inventrom</div>

        <div class="socials">
            <a href="https://www.facebook.com/boltiot" target="_blank">
                <span class="social">
                    <img src="facebook.png" alt="">
                </span>
            </a>
        
            <a href="https://twitter.com/boltiot" target="_blank">
                <span>
                    <img src="twitter.png" alt="">
                </span>
            </a>
        
            <a href="https://instagram.com/boltiot" target="_blank">
                <span>
                    <img src="instagram.png" alt="">
                </span>
            </a>
        </div>

        <a href="#top">
            <div class="up">
                <span>
                    <img src="double-up-arrow.png" alt="">
                </span>
            </div>
        </a>
    </div>


</body>
</html>


