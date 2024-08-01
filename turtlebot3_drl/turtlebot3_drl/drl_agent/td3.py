import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn

from ..common.manualaction import ManualAction
from ..common.settings import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY, ENABLE_IMITATE_ACTION
from ..common.ounoise import OUNoise
from ..common import utilities as util

from .off_policy_agent import OffPolicyAgent, Network

if ENABLE_IMITATE_ACTION:
    from ..common.storagemanager import StorageManager

import queue
import math

LINEAR = 0
ANGULAR = 1

# Reference for network structure: https://arxiv.org/pdf/2102.10711.pdf
# https://github.com/hanlinniu/turtlebot3_ddpg_collision_avoidance/blob/main/turtlebot_ddpg/scripts/original_ddpg/ddpg_network_turtlebot3_original_ddpg.py
# https://github.com/djbyrne/TD3
# Modified by: Seunghyeop
# Description: This code has been modified to train the Turtlebot3 Waffle_pi model.
# TD3 with LSTM & CNN 1 lstm parallel architecture

class TensorQueue():
    def __init__(self, maxlen=None, init=0, gpu='cpu'):
        self.maxlen = maxlen
        if maxlen == None:
            self.maxlen = math.inf
        self.gpu = gpu
        self.init = init
        self.queue = torch.zeros(self.init).unsqueeze(0).to(device=self.gpu)

    def put(self, val):
        if self.queue.size()[0] == 0:
            self.queue = val.unsqueeze(0)
        elif self.queue.size()[0] >= self.maxlen:
            self.get()
            torch.cat((self.queue, val.unsqueeze(0)), dim=0)
        else:
            torch.cat((self.queue, val.unsqueeze(0)), dim=0)

    def get(self):
        if self.queue.size()[0] <= 1:
            self.queue = torch.zeros(self.init).unsqueeze(0).to(device=self.gpu)
            return self.queue
        self.queue = self.queue[0:, :]
        return self.queue

class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)
        self.state_size = state_size - 6
        self.dev = util.check_gpu()
        self.state_history = TensorQueue(100, state_size, self.dev) #queue.Queue(hidden_size)
        # --- define layers here ---
        self.linear = nn.Sequential(
            # self.fa1 = nn.Linear(state_size - 6, hidden_size // 2 ** 2)
            nn.Linear(state_size - 6, hidden_size),
            nn.SiLU(),
            nn.Linear(hidden_size, int(hidden_size * 2)),
            nn.SiLU(),
            nn.Linear(int(hidden_size * 2), int(hidden_size * 2)),
            nn.SiLU(),
            # self.fa4 = nn.Linear(int(hidden_size * 2), hidden_size // 2 ** 2) # 128
            nn.Linear(int(hidden_size * 2), hidden_size // 2 ** 2), # 128
            nn.Sigmoid()
        )

        self.final_lin = nn.Sequential(
            # nn.Linear(int(hidden_size * 2), hidden_size // 2 ** 1),
            nn.Linear(state_size + hidden_size, hidden_size // 2 ** 1),
            nn.LayerNorm(hidden_size // 2 ** 1),
            nn.SiLU(),
            nn.Linear(int(hidden_size / 2 ** 1), hidden_size // 2 ** 2),
            nn.LayerNorm(hidden_size // 2 ** 2),
            nn.SiLU(),
            nn.Linear(hidden_size // 2 ** 2, hidden_size // 2 ** 3),
            nn.LayerNorm(hidden_size // 2 ** 3),
            nn.SiLU(),
            nn.Linear(hidden_size // 2 ** 3, action_size)
            # self.fa8 = nn.Linear(int(hidden_size / 2 ** 1), action_size)
        )

        # --- conv layers for feature extraction ---
        self.conv_iter = 3
        self.pooling_kernel_size = 2
        inner_channel_size = 2 ** 5
        fc_size = int(state_size / (self.pooling_kernel_size ** self.conv_iter)) * inner_channel_size

        self.conv = nn.Sequential(
            nn.Conv1d(1, inner_channel_size, 4, padding='same', padding_mode='circular'),
            # nn.BatchNorm1d(self.state_size),
            nn.SiLU(),
            nn.MaxPool1d(self.pooling_kernel_size),
            nn.Conv1d(inner_channel_size, inner_channel_size, 4, padding='same', padding_mode='circular'),
            # nn.BatchNorm1d(self.state_size // 2 ** 1),
            nn.SiLU(),
            nn.MaxPool1d(self.pooling_kernel_size),
            nn.Conv1d(inner_channel_size, inner_channel_size, 4, padding='same', padding_mode='circular'),
            # nn.BatchNorm1d(self.state_size // 2 ** 2),
            nn.Sigmoid(),
            nn.MaxPool1d(self.pooling_kernel_size)
        )

        # self.conv_fc = nn.Linear(fc_size, int(hidden_size / 2 ** 2)) # 128
        self.conv_fc = nn.Linear(fc_size, hidden_size // 2 ** 2)

        # --- lstm ---
        # self.lstm = nn.LSTM(input_size=320 + state_size, hidden_size=hidden_size // 2, num_layers=2)
        self.lstm1 = nn.LSTM(input_size=state_size, hidden_size=hidden_size // 2 ** 2, num_layers=1)
        self.lstm2 = nn.LSTM(input_size=hidden_size // 2 ** 2, hidden_size=hidden_size // 2 ** 2, num_layers=1)
        self.lstm3 = nn.LSTM(input_size=hidden_size // 2 ** 2, hidden_size=hidden_size // 2 ** 2, num_layers=1)
        # self.lstmfc = nn.Linear()

        # Env NN
        self.opfa = nn.Sequential(
            # self.opfa1 = nn.Linear(6, int(hidden_size / 2 ** 3)),
            nn.Linear(6, int(hidden_size / 2 ** 2)),
            nn.SiLU(),
            # self.opfa2 = nn.Linear(int(hidden_size / 2 ** 2), int(hidden_size / 2 ** 3)), # 64
            nn.Linear(int(hidden_size / 2 ** 2), hidden_size // 2 ** 2),
            nn.Sigmoid()
            # self.opfa3 = nn.Linear(int(hidden_size / 2 ** 2), int(hidden_size / 2 ** 3)),
        )

        self.dropout = nn.Dropout(0.5)

        self.layer_norm1 = nn.LayerNorm(hidden_size)
        self.layer_norm1_1 = nn.LayerNorm(int(hidden_size / 2) + int(hidden_size / 2 ** 3))
        self.layer_norm2 = nn.LayerNorm(int(hidden_size / 2 ** 1))
        self.layer_norm3 = nn.LayerNorm(int(hidden_size / 2 ** 2))
        self.layer_norm4 = nn.LayerNorm(int(hidden_size / 2 ** 3))

        # Custom Activation fucntion
        self.silu = nn.SiLU()

        self.apply(super().init_weights)

    def forward(self, states, visualize=False):
        # --- define forward pass here ---
        states = self.state_filter(states)
        if len(states.size()) == 2:
            lidar_states = states[:, :360]
            lidar_features = lidar_states.unsqueeze(dim=1)
            env_states = states[:, -6:]
            cat_dim = 1
            # hidd = self.hidden_train
        else:
            lidar_states = states[:360]
            lidar_features = torch.unsqueeze(lidar_states, dim=0)
            env_states = states[-6:]
            cat_dim = 0
            # hidd = self.hidden_real

        x = self.linear(lidar_states)

        feature = self.conv(lidar_features)
        feature = torch.flatten(feature, start_dim=cat_dim)
        feature = torch.sigmoid(self.conv_fc(feature))

        opx = self.opfa(env_states)

        if cat_dim == 1:
            lx = states.unsqueeze(cat_dim).repeat(1, states.size()[0], 1)
        else:
            # if self.state_history.full():
            #     self.state_history.get()
            self.state_history.put(states)
            lx = self.state_history.queue.unsqueeze(cat_dim)

        lx, hid = self.lstm1(lx)
        lx = self.silu(lx)
        lx, hid = self.lstm2(lx, hid)
        lx = self.silu(lx)
        lx, _ = self.lstm3(lx, hid)
        lx = torch.sigmoid(lx)[:, -1, :]
        lx = torch.flatten(lx, start_dim=cat_dim).squeeze(cat_dim)
        x = torch.cat((states, x, feature, lx, opx), cat_dim)

        action = torch.tanh(self.final_lin(x))

        # -- define layers to visualize here (optional) ---
        if visualize and self.visual:
            self.visual.update_layers(states, action, [x, x], [self.fa1.bias, self.fa2.bias])
        # -- define layers to visualize until here ---
        return action

    def state_filter(self, state):
        if len(state.size()) == 2 and state.size()[1] == 364:
            cat_dim = 1
            lidar_states = state[:, :self.state_size]
            env_states = state[:, -4:]
            lidar_states = (torch.exp((torch.ones_like(lidar_states) - lidar_states) * 4) - 1) / (torch.exp(torch.ones_like(lidar_states) * 4) - 1)
            max_values, max_indices = torch.max(lidar_states, dim=cat_dim)
            max_indices = max_indices / 360
            max_val_index = torch.stack((max_values, max_indices), dim=cat_dim)
            state = torch.cat((lidar_states, max_val_index, env_states), dim=cat_dim)

        elif len(state.size()) == 1 and state.size()[0] == 364:
            cat_dim = 0
            lidar_states = state[:self.state_size]
            env_states = state[-4:]
            lidar_states = (torch.exp((torch.ones_like(lidar_states) - lidar_states) * 4) - 1) / (torch.exp(torch.ones_like(lidar_states) * 4) - 1)
            max_values, max_indices = torch.max(lidar_states, dim=cat_dim)
            max_indices = max_indices / 360
            max_val_index = torch.stack((max_values, max_indices), dim=cat_dim)
            state = torch.cat((lidar_states, max_val_index, env_states), dim=cat_dim)
        return state

class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)
        self.state_size = state_size - 6
        self.l1 = nn.Linear(state_size, int(hidden_size / 2))
        self.l2 = nn.Linear(action_size, int(hidden_size / 2))
        self.l3 = nn.Linear(hidden_size, hidden_size)
        self.l4 = nn.Linear(hidden_size, 1)

        # Q2
        # --- define layers here ---
        self.l5 = nn.Linear(state_size, int(hidden_size / 2))
        self.l6 = nn.Linear(action_size, int(hidden_size / 2))
        self.l7 = nn.Linear(hidden_size, hidden_size)
        self.l8 = nn.Linear(hidden_size, 1)

        self.silu = nn.SiLU()

        self.apply(super().init_weights)

    def forward(self, states, actions):
        states = self.state_filter(states)
        xs = self.silu(self.l1(states))
        xa = self.silu(self.l2(actions))
        # print(xs.size(), ', ', xa.size())
        x = torch.cat((xs, xa), dim=1)
        x = self.silu(self.l3(x))
        x1 = self.l4(x)

        xs = self.silu(self.l5(states))
        xa = self.silu(self.l6(actions))
        x = torch.cat((xs, xa), dim=1)
        x = self.silu(self.l7(x))
        x2 = self.l8(x)

        return x1, x2

    def Q1_forward(self, states, actions):
        states = self.state_filter(states)
        xs = self.silu(self.l1(states))
        xa = self.silu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = self.silu(self.l3(x))
        x1 = self.l4(x)
        return x1

    def state_filter(self, state):
        if len(state.size()) == 2 and state.size()[1] == 364:
            cat_dim = 1
            lidar_states = state[:, :self.state_size]
            env_states = state[:, -4:]
            lidar_states = (torch.exp((torch.ones_like(lidar_states) - lidar_states) * 4) - 1) / (torch.exp(torch.ones_like(lidar_states) * 4) - 1)
            max_values, max_indices = torch.max(lidar_states, dim=cat_dim)
            max_indices = max_indices / 360
            max_val_index = torch.stack((max_values, max_indices), dim=cat_dim)
            state = torch.cat((lidar_states, max_val_index, env_states), dim=cat_dim)

        elif len(state.size()) == 1 and state.size()[0] == 364:
            cat_dim = 0
            lidar_states = state[:self.state_size]
            env_states = state[-4:]
            lidar_states = (torch.exp((torch.ones_like(lidar_states) - lidar_states) * 4) - 1) / (torch.exp(torch.ones_like(lidar_states) * 4) - 1)
            max_values, max_indices = torch.max(lidar_states, dim=cat_dim)
            max_indices = max_indices / 360
            max_val_index = torch.stack((max_values, max_indices), dim=cat_dim)
            state = torch.cat((lidar_states, max_val_index, env_states), dim=cat_dim)
        return state



class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        self.manual_action = ManualAction()

        # DRL parameters
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)

        # TD3 parameters
        self.policy_noise   = POLICY_NOISE
        self.noise_clip     = POLICY_NOISE_CLIP
        self.policy_freq    = POLICY_UPDATE_FREQUENCY

        self.last_actor_loss = 0

        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = self.create_network(Actor, 'target_actor')
        self.actor_optimizer = self.create_optimizer(self.actor)
        # self.actor_lr_scheduler = self.create_lr_scheduler(self.actor_optimizer)

        self.critic = self.create_network(Critic, 'critic')
        self.critic_target = self.create_network(Critic, 'target_critic')
        self.critic_optimizer = self.create_optimizer(self.critic)
        # self.critic_lr_scheduler = self.create_lr_scheduler(self.critic_optimizer)

        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)

        if ENABLE_IMITATE_ACTION:
            self.sm = StorageManager('ddpg', 'examples_waffle_pi/ddpg_0_stage_10', 15400, self.device, '10')
            self.imit_model = self.sm.load_model()
            self.imit_model.device = self.device
            self.sm.load_weights(self.imit_model.networks)


    def get_action(self, state, is_training, step, visualize=False):
        state = torch.from_numpy(np.asarray(state, np.float32)).to(dtype=torch.float32, device=self.device)
        action = self.actor(state, visualize)
        if is_training:
            noise = torch.from_numpy(copy.deepcopy(self.noise.get_noise(step))).to(dtype=torch.float32, device=self.device)
            action = torch.clamp(torch.add(action, noise), -1.0, 1.0)
        return action.detach().cpu().data.numpy().tolist()

    def get_action_random(self):
        return [np.clip(np.random.uniform(0, 1.0), -1.0, 1.0), np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0)]

    def get_action_manual(self):
        return self.manual_action.get_action()

    def get_action_imitate(self, state):
        visualize=False
        state = torch.from_numpy(np.asarray(state, np.float32)).to(dtype=torch.float32, device=self.device)
        action = self.imit_model.actor(state, visualize)
        return action.detach().cpu().data.numpy().tolist()


    def train(self, state, action, reward, state_next, done):
        noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
        action_next = (self.actor_target(state_next) + noise).clamp(-1.0, 1.0)
        Q1_next, Q2_next = self.critic_target(state_next, action_next)
        Q_next = torch.min(Q1_next, Q2_next)

        Q_target = reward + (1 - done) * self.discount_factor * Q_next
        Q1, Q2 = self.critic(state, action)

        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)
        self.critic_optimizer.zero_grad()
        loss_critic.backward(retain_graph=True)
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=2.0, norm_type=2)
        self.critic_optimizer.step()
        # self.critic_lr_scheduler.step()

        if self.iteration % self.policy_freq == 0:
            # optimize actor
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            loss_actor.backward(retain_graph=True)
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
            self.actor_optimizer.step()
            # self.actor_lr_scheduler.step()

            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)
            self.last_actor_loss = loss_actor.mean().detach().cpu()
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]