import streamlit as st
import requests
import matplotlib.pyplot as plt
import time
import os

st.markdown("""
    <style>
        [data-testid="stSidebar"] {
            background-color: #ffffff;
            background-image: linear-gradient(#2e7bcf, #2ca58d);
            color: black;
        }
        .header-text {
            text-align: center;
            color: #ffffff;
            background-color: #2e7bcf;
            padding: 13px; 
            border-radius: 15px;
            margin-bottom: 10px; 
            border: 2px solid black;
            font-size: 16px;
        }
        [data-testid="stMetricValue"] {
            font-size: 16px;
        }
        [data-testid="stMetricLabel"] {
            font-size: 12px;
        }
    </style>
    """, unsafe_allow_html=True)

# Sidebar
path = st.sidebar.selectbox("Pilih Lintasan", ["Lintasan A", "Lintasan B"])
start_monitoring_button = st.sidebar.button("START BUTTON", key="start_monitoring_button")

col1, col2, col3, col4 = st.columns([1,2,2,1])
with col1:
    st.image('logo.jpeg', width=100)
with col2:
    st.markdown("<h6 class='header-text'> BARELANG MARINE ROBOTICS TEAM </h6>", unsafe_allow_html=True)
with col3:
    st.markdown("<h6 class='header-text'> POLITEKNIK NEGERI BATAM </h6>", unsafe_allow_html=True)
with col4:
    st.image('Nereus Polos.png', width=110)


st.markdown("<h6 class='header-text'>GEO-TAG INFOS</h6>", unsafe_allow_html=True)
col1, col2, col3, col4 = st.columns(4)
day_placeholder = col1.metric("Day", "Loading...")
date_placeholder = col2.metric("Date", "Loading...")
time_placeholder = col3.metric("Time", "Loading...")
coordinate_placeholder = col4.metric("Coordinate", "Loading...")
col1, col2, col3, col4 = st.columns(4)
sog_knot_placeholder = col1.metric("SOG [Knot]", "Loading...")
sog_kmh_placeholder = col2.metric("SOG [Km/h]", "Loading...")
cog_placeholder = col3.metric("COG", "Loading...")
position_placeholder = col4.metric("Position [x,y]", "Loading...")

st.markdown("<h5 class='header-text'>TRAJECTORY MAP</h5>", unsafe_allow_html=True)

# Backend URL
FLASK_URL = 'http://127.0.0.1:5000/random-data'


def data_backend():
    try:
        response = requests.get(FLASK_URL)
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        st.error(f"Error fetching data: {e}")
        return None
def judul_hasil_foto():
    col1, col2 = st.columns(2)
    with col1 :
        st.markdown("<h6 class='header-text'>Under Water Picture</h6>", unsafe_allow_html=True)
    with col2 :
        st.markdown("<h6 class='header-text'>Water Surface Picture</h6>", unsafe_allow_html=True)


def koordinat_kartesius(path):
    
    fig, ax = plt.subplots(figsize=(13, 13))
    ax.set_xlim(0, 2500)
    ax.set_ylim(0, 2500)
    ax.set_xticks(range(0, 2600, 100))
    ax.set_yticks(range(0, 2600, 100))
    ax.grid(True)
    if path == "Lintasan A":
        ##Titik nol : x = 2185, y = 115
        rectangle = plt.Rectangle((2100, 65), 170, 100, color='red', fill=True)
        red_positions, green_positions = posisi_floating_ball("A")

        rectangle = plt.Rectangle((2100, 65), 170, 100, color='red', fill=True)
        ax.add_patch(rectangle)

        blue_rectangle = plt.Rectangle((520, 300), 100, 50, color='blue', fill=True)
        ax.add_patch(blue_rectangle)

        small_green_rectangle = plt.Rectangle((300, 620), 100, 50, color='green', fill=True)
        ax.add_patch(small_green_rectangle)

    elif path == "Lintasan B":
        ##Titik nol : x = 335, y = 115
        rectangle = plt.Rectangle((250, 65), 170, 100, color='green', fill=True)
        red_positions, green_positions = posisi_floating_ball("B")
        
        rectangle = plt.Rectangle((250, 65), 170, 100, color='green', fill=True)
        ax.add_patch(rectangle)

        blue_rectangle = plt.Rectangle((1880, 300), 100, 50, color='blue', fill=True)
        ax.add_patch(blue_rectangle)

        small_green_rectangle = plt.Rectangle((2100, 620), 100, 50, color='green', fill=True)
        ax.add_patch(small_green_rectangle)
    else:
        gambar_lintasan_lomba()
        return None, None

    ax.add_patch(rectangle)
    for pos in red_positions:
        ax.add_patch(plt.Circle(pos, 25, color='red', fill=True))
    for pos in green_positions:
        ax.add_patch(plt.Circle(pos, 25, color='green', fill=True))

    return fig, ax

def hasil_foto_surface():
    surface_dir = os.path.expanduser("~/ros_ws/src/train_asv/src/surface_box")
    surface_file = "sbox_1.jpg" 
    surface_path = os.path.join(surface_dir, surface_file)
    
    if os.path.isfile(surface_path):
        with image_placeholder.container():
            col1, _ = st.columns([1, 1])
            with col1:
                st.image(surface_path, caption="Surface Water Box", use_column_width=True)
    else:
        return None, None
        
def hasil_foto_underwater():
    underwater_dir = os.path.expanduser("~/ros_ws/src/train_asv/src/underwater_box")
    underwater_file = "ubox_1.jpg"  
    underwater_path = os.path.join(underwater_dir, underwater_file)
    
    if os.path.isfile(underwater_path):
        with image_placeholder.container():
            _, col2 = st.columns([1, 1])
            with col2:
                st.image(underwater_path, caption="Under Water Box", use_column_width=True)
    else:
        return None, None

def gambar_lintasan_lomba():
    col1, col2 = st.columns(2)
    with col1:
        st.image('lintasanA.jpeg', caption='Lintasan A')
    with col2:
        st.image('lintasanB.jpeg', caption='Lintasan B')

def posisi_floating_ball(path):
    if path == "A":
        green_positions = [(330, 960), (330, 1310), (450, 1715), (1040, 2230), (1200, 2230),
                         (1360, 2230), (1520, 2230), (2305, 1465), (2160, 1160), (2240, 855)]
        red_positions = [(180, 960), (180, 1310), (300, 1715), (1040, 2100), (1200, 2100),
                           (1360, 2100), (1520, 2100), (2175, 1465), (2030, 1160), (2110, 855)]
    elif path == "B":
        red_positions = [(390, 855), (470, 1160), (325, 1465), (980, 2100), (1140, 2100),
                         (1300, 2100), (1460, 2100), (2220, 1715), (2320, 1310), (2320, 960)]
        green_positions = [(260, 855), (340, 1160), (195, 1465), (980, 2230), (1140, 2230),
                           (1300, 2230), (1460, 2230), (2070, 1715), (2170, 1310), (2170, 960)]
    return red_positions, green_positions


plot_placeholder = st.empty()
image_placeholder = st.empty()

fig, ax = koordinat_kartesius(path)

trajectory_x = []
trajectory_y = []
trajectory_line, = ax.plot([], [], color='blue', linestyle='-', marker='o', markersize=3)


def update_plot():
#fungsi update data
    global trajectory_x, trajectory_y
    data = data_backend()
    if data:
        try:
            if path == "Lintasan B":
                x = data.get('x') + 335
                y = data.get('y') + 115
            
            else:
                x = data.get('x') + 2185
                y = data.get('y') + 115


            sog_knot = data.get('sog_knot')
            sog_kmh = data.get('sog_kmh')
            cog = data.get('cog')
            day = data.get('day')
            date = data.get('date')
            time_value = data.get('time')
            coordinate = data.get('coordinate')

            trajectory_x.append(x)
            trajectory_y.append(y)
            trajectory_line.set_data(trajectory_x, trajectory_y)
            ax.relim()
            ax.autoscale_view()

            sog_knot_placeholder.metric("SOG [Knot]", f"{sog_knot} kt")
            sog_kmh_placeholder.metric("SOG [Km/h]", f"{sog_kmh} km/h")
            cog_placeholder.metric("COG", f"{cog}°")
            day_placeholder.metric("Day", day)
            date_placeholder.metric("Date", date)
            time_placeholder.metric("Time", time_value)
            coordinate_placeholder.metric("Coordinate", coordinate)
            position_placeholder.metric("Position [x,y]",f"{x}, {y}")

            plot_placeholder.pyplot(fig)

        except (TypeError, KeyError) as e:
            st.error(f"Error processing data: {e}")
    else:
        st.warning("No data received.")
        

# Main loop
if start_monitoring_button:
    st.text("Monitoring started...")
    while True:
        update_plot()
        hasil_foto_surface()
        hasil_foto_underwater()
        time.sleep(1)
        
else:
    gambar_lintasan_lomba()