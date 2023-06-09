/********************************************************************************************************
 * @file     ChooseAndAddDeviceVC.m
 *
 * @brief    for TLSR chips
 *
 * @author   Telink, 梁家誌
 * @date     2019/7/4
 *
 * @par     Copyright (c) [2021], Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/

#import "ChooseAndAddDeviceVC.h"
#import "MeshOTAItemCell.h"
#import "UIViewController+Message.h"
#import "UIButton+extension.h"

typedef enum : NSUInteger {
    AddStateScan,
    AddStateProvisioning,
    AddStateProvisionFail,
    AddStateKeybinding,
    AddStateKeybound,
    AddStateUnbound,
} AddState;
@interface AddDeviceStateModel : NSObject
@property (nonatomic,strong) CBPeripheral *peripheral;
@property (nonatomic,assign) AddState state;
@end

@implementation AddDeviceStateModel
- (BOOL)isEqual:(id)object{
    if ([object isKindOfClass:[AddDeviceStateModel class]]) {
        return [_peripheral.identifier.UUIDString isEqualToString:((AddDeviceStateModel *)object).peripheral.identifier.UUIDString];
    } else {
        return NO;
    }
}
@end

@interface ChooseAndAddDeviceVC ()<UITableViewDelegate,UITableViewDataSource>
@property (weak, nonatomic) IBOutlet UITableView *tableView;
@property (weak, nonatomic) IBOutlet UIButton *addButton;
@property (weak, nonatomic) IBOutlet UIButton *scanButton;
@property (nonatomic,strong) NSMutableArray <AddDeviceStateModel *>*allDevices;
@property (nonatomic, strong) NSMutableArray <AddDeviceStateModel *>*selectDevices;

@end

@implementation ChooseAndAddDeviceVC

- (IBAction)clickScan:(UIButton *)sender {
    self.allDevices = [NSMutableArray array];
    self.selectDevices = [NSMutableArray array];
    [self.tableView reloadData];
    [self userAbled:NO];

    __weak typeof(self) weakSelf = self;
    [SDKLibCommand stopMeshConnectWithComplete:^(BOOL successful) {
        if (successful) {
            TeLogDebug(@"close success.");
            //示范代码：只扫描1827的设备
            [SDKLibCommand scanUnprovisionedDevicesWithResult:^(CBPeripheral * _Nonnull peripheral, NSDictionary<NSString *,id> * _Nonnull advertisementData, NSNumber * _Nonnull RSSI, BOOL unprovisioned) {
                TeLogInfo(@"==========peripheral=%@,advertisementData=%@,RSSI=%@,unprovisioned=%d",peripheral,advertisementData,RSSI,unprovisioned);
                if (unprovisioned) {
                    AddDeviceStateModel *model = [[AddDeviceStateModel alloc] init];
                    model.peripheral = peripheral;
                    model.state = AddStateScan;
                    if (![weakSelf.allDevices containsObject:model]) {
                        [weakSelf.allDevices addObject:model];
                        [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                    }
                }
            }];
            //示范代码：同时扫描1827和1828的设备
//            [SDKLibCommand scanWithServiceUUIDs:@[[CBUUID UUIDWithString:kPROXYService],[CBUUID UUIDWithString:kPBGATTService]] checkNetworkEnable:NO result:^(CBPeripheral *peripheral, NSDictionary<NSString *,id> *advertisementData, NSNumber *RSSI, BOOL unprovisioned) {
//                if (unprovisioned) {
//                    AddDeviceStateModel *model = [[AddDeviceStateModel alloc] init];
//                    model.peripheral = peripheral;
//                    model.state = AddStateScan;
//                    if (![weakSelf.allDevices containsObject:model]) {
//                        [weakSelf.allDevices addObject:model];
//                        [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                    }
//                } else {
//                    AddDeviceStateModel *model = [[AddDeviceStateModel alloc] init];
//                    model.peripheral = peripheral;
//                    model.state = AddStateKeybound;
//                    if (![weakSelf.allDevices containsObject:model]) {
//                        [weakSelf.allDevices addObject:model];
//                        [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                    }
//                }
//            }];
            dispatch_async(dispatch_get_main_queue(), ^{
                [NSObject cancelPreviousPerformRequestsWithTarget:weakSelf selector:@selector(scanFinish) object:nil];
                [weakSelf performSelector:@selector(scanFinish) withObject:nil afterDelay:5.0];
            });
        } else {
            TeLogDebug(@"close fail.");
            [weakSelf userAbled:YES];
        }
    }];
}

- (IBAction)clickAddDevice:(UIButton *)sender {
    if (self.allDevices.count == 0) {
        [self showTips:@"please scan device!"];
        return;
    }
    NSMutableArray *unprovisionList = [NSMutableArray array];
    NSArray *selectDevices = [NSArray arrayWithArray:self.selectDevices];
    for (AddDeviceStateModel *model in selectDevices) {
        if (model.state == AddStateScan) {
            [unprovisionList addObject:model];
        }
    }
    if (unprovisionList.count == 0) {
        [self showTips:@"please choose unprovision device!"];
        return;
    }
    self.selectDevices = [NSMutableArray arrayWithArray:unprovisionList];
    
    [self userAbled:NO];
    NSData *key = [SigDataSource.share curNetKey];
    if (SigDataSource.share.curNetkeyModel.phase == distributingKeys) {
        if (SigDataSource.share.curNetkeyModel.oldKey) {
            key = [LibTools nsstringToHex:SigDataSource.share.curNetkeyModel.oldKey];
        }
    }

    __weak typeof(self) weakSelf = self;
    [SDKLibCommand stopMeshConnectWithComplete:^(BOOL successful) {
        if (successful) {
            NSOperationQueue *operationQueue = [[NSOperationQueue alloc] init];
            [operationQueue addOperationWithBlock:^{
                for (AddDeviceStateModel *model in selectDevices) {
                    dispatch_semaphore_t semaphore = dispatch_semaphore_create(0);
                    CBPeripheral *peripheral = model.peripheral;
                    UInt16 provisionAddress = [SigDataSource.share provisionAddress];
                    if (provisionAddress == 0) {
                        TeLogDebug(@"warning: address has run out.");
                        [weakSelf userAbled:YES];
                        return;
                    }
                    NSNumber *type = [[NSUserDefaults standardUserDefaults] valueForKey:kKeyBindType];
                    model.state = AddStateProvisioning;
                    [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                    
                    //选择添加新增逻辑：判断本地是否存在该UUID的OOB数据，存在则使用static OOB添加，不存在则使用no OOB添加。
                    SigScanRspModel *rspModel = [SigDataSource.share getScanRspModelWithUUID:peripheral.identifier.UUIDString];
                    SigOOBModel *oobModel = [SigDataSource.share getSigOOBModelWithUUID:rspModel.advUuid];
                    ProvisionType provisionType = ProvisionType_NoOOB;
                    NSData *staticOOBData = nil;
                    if (oobModel && oobModel.OOBString && (oobModel.OOBString.length == 32 || oobModel.OOBString.length == 64)) {
                        provisionType = ProvisionType_StaticOOB;
                        staticOOBData = [LibTools nsstringToHex:oobModel.OOBString];
                    }
                    if (rspModel.advOobInformation.supportForCertificateBasedProvisioning) {
                        if (kExistCertificateBasedProvision) {
#ifdef kExist
                            [SDKLibCommand startCertificateBasedWithAddress:provisionAddress networkKey:key netkeyIndex:SigDataSource.share.curNetkeyModel.index appkeyModel:SigDataSource.share.curAppkeyModel peripheral:peripheral provisionType:provisionType staticOOBData:staticOOBData keyBindType:type.integerValue productID:0 cpsData:nil provisionSuccess:^(NSString * _Nonnull identify, UInt16 address) {
                                model.state = AddStateKeybinding;
                                [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                            } provisionFail:^(NSError * _Nullable error) {
                                model.state = AddStateProvisionFail;
                                [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                                dispatch_semaphore_signal(semaphore);
                            } keyBindSuccess:^(NSString * _Nonnull identify, UInt16 address) {
                                model.state = AddStateKeybound;
                                SigNodeModel *node = [SigDataSource.share getNodeWithAddress:address];
                                if (node && node.isRemote) {
                                    [node addDefaultPublicAddressToRemote];
                                    [SigDataSource.share saveLocationData];
                                }
                                [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                                dispatch_semaphore_signal(semaphore);
                            } keyBindFail:^(NSError * _Nullable error) {
                                model.state = AddStateUnbound;
                                [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                                dispatch_semaphore_signal(semaphore);
                            }];
#endif

                        } else {
                            TeLogInfo(@"Device is certificate-based device, but this SDK is not support certificate-based provision.");
                            model.state = AddStateProvisionFail;
                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                            dispatch_semaphore_signal(semaphore);
                        }
                    } else {
//                        //fastbind 获取cpsData1：PID为1或者7，SDK内置了这两个PID的cpsData，当接口是fastBind且cpsData为空，则使用PID查找cpsData，查找不到则默认使用PID为1的cpsData。
//    //                    UInt16 productID = 1;
//    //                    DeviceTypeModel *deviceType = [SigDataSource.share getNodeInfoWithCID:kCompanyID PID:productID];
//    //                    NSData *cpsData = deviceType.defaultCompositionData.parameters;
//                        //fastbind 获取cpsData2：当接口是fastBind且传入cpsData，则无论PID为多少，直接使用cpsData里面的PID。
//                        UInt16 productID = 0;
//                        NSData *cpsData = [NSData dataWithBytes:CTByte length:sizeof(CTByte)];
//                        //fastbind接口调用如下：
//                        [SDKLibCommand startAddDeviceWithNextAddress:provisionAddress networkKey:key netkeyIndex:SigDataSource.share.curNetkeyModel.index appkeyModel:SigDataSource.share.curAppkeyModel peripheral:peripheral provisionType:provisionType staticOOBData:staticOOBData keyBindType:KeyBindType_Fast productID:productID cpsData:cpsData provisionSuccess:^(NSString * _Nonnull identify, UInt16 address) {
//                            model.state = AddStateKeybinding;
//                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                        } provisionFail:^(NSError * _Nonnull error) {
//                            model.state = AddStateProvisionFail;
//                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                            dispatch_semaphore_signal(semaphore);
//                        } keyBindSuccess:^(NSString * _Nonnull identify, UInt16 address) {
//                            model.state = AddStateKeybound;
//                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                            dispatch_semaphore_signal(semaphore);
//                        } keyBindFail:^(NSError * _Nonnull error) {
//                            model.state = AddStateUnbound;
//                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
//                            dispatch_semaphore_signal(semaphore);
//                        }];
                        
                        [SDKLibCommand startAddDeviceWithNextAddress:provisionAddress networkKey:key netkeyIndex:SigDataSource.share.curNetkeyModel.index appkeyModel:SigDataSource.share.curAppkeyModel peripheral:peripheral provisionType:provisionType staticOOBData:staticOOBData keyBindType:type.integerValue productID:0 cpsData:nil provisionSuccess:^(NSString * _Nonnull identify, UInt16 address) {
                            model.state = AddStateKeybinding;
                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                        } provisionFail:^(NSError * _Nonnull error) {
                            model.state = AddStateProvisionFail;
                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                            dispatch_semaphore_signal(semaphore);
                        } keyBindSuccess:^(NSString * _Nonnull identify, UInt16 address) {
                            model.state = AddStateKeybound;
                            SigNodeModel *node = [SigDataSource.share getNodeWithAddress:address];
                            if (node && node.isRemote) {
                                [node addDefaultPublicAddressToRemote];
                                [SigDataSource.share saveLocationData];
                            }
                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                            dispatch_semaphore_signal(semaphore);
                        } keyBindFail:^(NSError * _Nonnull error) {
                            model.state = AddStateUnbound;
                            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
                            dispatch_semaphore_signal(semaphore);
                        }];
                    }
                    dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                }
                [SDKLibCommand startMeshConnectWithComplete:nil];
                dispatch_async(dispatch_get_main_queue(), ^{
                    [weakSelf.selectDevices removeAllObjects];
                    [weakSelf.tableView reloadData];
                    [weakSelf userAbled:YES];
                });
            }];
        }else{
            TeLogDebug(@"stop mesh fail.");

        }
    }];
}

- (void)scanFinish{
    [SDKLibCommand stopScan];
    [self userAbled:YES];
}

- (void)normalSetting{
    [super normalSetting];
    self.title = @"choose add device";
    self.tableView.tableFooterView = [[UIView alloc] initWithFrame:CGRectZero];
    [self.tableView registerNib:[UINib nibWithNibName:CellIdentifiers_MeshOTAItemCellID bundle:nil] forCellReuseIdentifier:CellIdentifiers_MeshOTAItemCellID];
    self.allDevices = [NSMutableArray array];
    self.selectDevices = [NSMutableArray array];
    [SDKLibCommand setBluetoothCentralUpdateStateCallback:nil];
}

- (void)viewWillAppear:(BOOL)animated{
    [super viewWillAppear:animated];
    self.tabBarController.tabBar.hidden = YES;
}

- (void)viewDidDisappear:(BOOL)animated{
    [super viewDidDisappear:animated];
    dispatch_async(dispatch_get_main_queue(), ^{
        [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(scanFinish) object:nil];
    });
    [SDKLibCommand stopScan];
}

- (void)userAbled:(BOOL)able{
    self.addButton.enabled = able;
    self.scanButton.enabled = able;
    self.tableView.userInteractionEnabled = able;
    self.addButton.backgroundColor = able ? kDefultColor : kDefultUnableColor;
    self.scanButton.backgroundColor = able ? kDefultColor : kDefultUnableColor;
}

- (void)showTips:(NSString *)message{
    __weak typeof(self) weakSelf = self;
    dispatch_async(dispatch_get_main_queue(), ^{
        [weakSelf showAlertSureWithTitle:@"Hits" message:message sure:^(UIAlertAction *action) {
            
        }];
    });
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath{
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifiers_MeshOTAItemCellID forIndexPath:indexPath];
    [self configureCell:cell forRowAtIndexPath:indexPath];
    return cell;
}

- (void)configureCell:(UITableViewCell *)cell forRowAtIndexPath:(NSIndexPath *)indexPath{
    MeshOTAItemCell *itemCell = (MeshOTAItemCell *)cell;
    __weak typeof(self) weakSelf = self;
    
    if (indexPath.row == 0) {
        itemCell.titleLabel.text = @"choose all";
        itemCell.selectButton.selected = self.selectDevices.count == self.allDevices.count;
        if (self.allDevices.count == 0) {
            itemCell.selectButton.selected = NO;
        }
        [itemCell.selectButton addAction:^(UIButton *button) {
            if (!button.selected) {
                weakSelf.selectDevices = [NSMutableArray array];
                for (AddDeviceStateModel *m in weakSelf.allDevices) {
                    if (m.state == AddStateScan) {
                        [weakSelf.selectDevices addObject:m];
                    }
                }
            } else {
                [weakSelf.selectDevices removeAllObjects];
            }
            [weakSelf.tableView performSelectorOnMainThread:@selector(reloadData) withObject:nil waitUntilDone:YES];
        }];
    }else{
        AddDeviceStateModel *model = self.allDevices[indexPath.row-1];
        SigScanRspModel *rsp = [SigDataSource.share getScanRspModelWithUUID:model.peripheral.identifier.UUIDString];
        NSString *state = @"";
        switch (model.state) {
            case AddStateScan:
                state = @"scan";
                break;
            case AddStateProvisioning:
                state = @"provisioning";
                break;
            case AddStateProvisionFail:
                state = @"provisionFail";
                break;
            case AddStateKeybinding:
                state = @"keybinding";
                break;
            case AddStateKeybound:
                state = @"keybound";
                break;
            case AddStateUnbound:
                state = @"unbound";
                break;
            default:
                break;
        }
        itemCell.certIcon.hidden = rsp.advOobInformation.supportForCertificateBasedProvisioning != 1;
        //旧显示方式：没有MacAddress则显示UUID
//        if (rsp.macAddress) {
//            itemCell.titleLabel.text = [NSString stringWithFormat:@"mac:%@ state:%@\ndeviceUuid:%@",rsp.macAddress,state,rsp.advUuid];
//        } else {
//            itemCell.titleLabel.text = [NSString stringWithFormat:@"uuid:%@ state:%@\ndeviceUuid:%@",model.peripheral.identifier.UUIDString,state,rsp.advUuid];
//        }
        //新显示方式：没有MacAddress则显示（null）
        itemCell.titleLabel.text = [NSString stringWithFormat:@"mac:%@ state:%@\ndeviceUuid:%@",rsp.macAddress,state,rsp.advUuid];
        if (self.selectDevices.count > 0) {
            itemCell.selectButton.selected = [self.selectDevices containsObject:model];
        } else {
            itemCell.selectButton.selected = NO;
        }
        
        [itemCell.selectButton addAction:^(UIButton *button) {
            if (model.state == AddStateScan) {
                if ([weakSelf.selectDevices containsObject:model]) {
                    [weakSelf.selectDevices removeObject:model];
                }else{
                    [weakSelf.selectDevices addObject:model];
                }
                [weakSelf.tableView reloadData];
            }
        }];
    }
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section{
    return self.allDevices.count + 1;
}

- (CGFloat)tableView:(UITableView *)tableView heightForRowAtIndexPath:(NSIndexPath *)indexPath{
    return 55.0;
}

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath{
    if (indexPath.row == 0) {
        MeshOTAItemCell *itemCell = [tableView cellForRowAtIndexPath:indexPath];
        if (!itemCell.selectButton.selected) {
            self.selectDevices = [NSMutableArray array];
            for (AddDeviceStateModel *m in self.allDevices) {
                if (m.state == AddStateScan) {
                    [self.selectDevices addObject:m];
                }
            }
        } else {
            [self.selectDevices removeAllObjects];
        }
    } else {
        AddDeviceStateModel *model = self.allDevices[indexPath.row - 1];
        if (model.state == AddStateScan) {
            if ([self.selectDevices containsObject:model]) {
                [self.selectDevices removeObject:model];
            }else{
                [self.selectDevices addObject:model];
            }
        }
    }
    [self.tableView reloadData];
}

-(void)dealloc{
    TeLogDebug(@"");
}

@end
